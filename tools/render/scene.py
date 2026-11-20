"""Path-traced renders of the exported meshes. Runs inside Blender 5.2, not in the uv env.

    flatpak run org.blender.Blender -b --factory-startup --python tools/render/scene.py \
        -- <manifest.json>

tools/render/meshes.py writes the manifest and one OBJ per solid (mm, assembly pose, exact
normals).
Lights, finishes, crop and the shadow catcher are ported from worxboard-ergo tools/render.py.
The pictures show a design that has not been printed.
"""

import json
import sys
from dataclasses import dataclass
from math import cos, radians, sin
from pathlib import Path

import bpy
from mathutils import Matrix, Vector

MM = 0.001
SAMPLES = 96
SENSOR_MM = 36.0
FRAME_FILL = 0.84
WORLD_STRENGTH = 0.45


@dataclass(frozen=True)
class Finish:
    colour: str
    roughness: float
    metallic: float = 0.0
    grain: float = 0.0
    grain_mm: float = 0.25
    coat: float = 0.0


FINISHES = {
    "petg": Finish("#55585d", 0.5, grain=0.35, grain_mm=0.2),  # space grey PETG
    "tpu": Finish("#121314", 0.8, grain=0.3, grain_mm=0.3),
    "steel": Finish("#c4c6c9", 0.35, metallic=1.0),
    "black_steel": Finish("#1b1c1e", 0.4, metallic=1.0),
    "brass": Finish("#b08d57", 0.3, metallic=1.0),
    "column": Finish("#b8bbbf", 0.3, metallic=1.0, grain=0.15, grain_mm=0.1),
    "camera": Finish("#1a1b1d", 0.55, grain=0.4, grain_mm=0.3),
    "zebra": Finish("#050505", 0.04, coat=1.0),
}


@dataclass(frozen=True)
class Light:
    position: tuple
    size: float
    watts: float


LIGHTS = (
    Light((-0.70, -0.30, 0.80), 1.0, 16.0),  # key: soft box above, camera left
    Light((0.70, -0.50, 0.75), 1.2, 5.0),  # fill
    Light((0.35, 0.80, 0.55), 0.5, 9.0),  # rim
)


def linear(colour):
    channels = [int(colour[i:i + 2], 16) / 255 for i in (1, 3, 5)]
    return [c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in channels] + [1.0]


def material_of(name):
    if name in bpy.data.materials:
        return bpy.data.materials[name]
    finish = FINISHES[name]
    material = bpy.data.materials.new(name)
    material.use_nodes = True
    nodes, links = material.node_tree.nodes, material.node_tree.links
    surface = nodes["Principled BSDF"]
    surface.inputs["Base Color"].default_value = linear(finish.colour)
    surface.inputs["Roughness"].default_value = finish.roughness
    surface.inputs["Metallic"].default_value = finish.metallic
    surface.inputs["Coat Weight"].default_value = finish.coat
    if finish.grain:
        noise, bump = nodes.new("ShaderNodeTexNoise"), nodes.new("ShaderNodeBump")
        noise.inputs["Scale"].default_value = 1 / (finish.grain_mm * MM)
        bump.inputs["Strength"].default_value = finish.grain
        bump.inputs["Distance"].default_value = finish.grain_mm * MM / 10
        links.new(nodes.new("ShaderNodeTexCoord").outputs["Object"], noise.inputs["Vector"])
        links.new(noise.outputs["Fac"], bump.inputs["Height"])
        links.new(bump.outputs["Normal"], surface.inputs["Normal"])
    return material


def import_mesh(entry):
    """OBJ from meshes.py: mm, Z up, the CAD surface's own normals at every node."""
    before = set(bpy.data.objects)
    bpy.ops.wm.obj_import(filepath=entry["mesh"], global_scale=MM, forward_axis="Y",
                          up_axis="Z")
    placed = (set(bpy.data.objects) - before).pop()
    placed.name = entry["name"]
    mesh = placed.data
    mesh.materials.clear()
    mesh.materials.append(material_of(entry["finish"]))
    placed["groups"] = ",".join(entry["groups"])
    placed["finish"] = entry["finish"]
    return placed


def set_up_renderer(scene):
    scene.render.engine, scene.render.film_transparent = "CYCLES", True
    scene.cycles.samples, scene.cycles.use_denoising = SAMPLES, True
    scene.view_settings.view_transform = "AgX"
    scene.view_settings.look = "AgX - Medium High Contrast"
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGBA"


def studio_world(scene):
    world = bpy.data.worlds.new("studio")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs["Strength"].default_value = WORLD_STRENGTH
    scene.world = world


def zebra_world(scene):
    """Horizontal light bands all round: their reflections show the curvature flow (G2)."""
    world = bpy.data.worlds.new("zebra")
    world.use_nodes = True
    nodes, links = world.node_tree.nodes, world.node_tree.links
    coords, wave = nodes.new("ShaderNodeTexCoord"), nodes.new("ShaderNodeTexWave")
    wave.wave_type, wave.bands_direction = "BANDS", "Z"
    wave.inputs["Scale"].default_value = 9.0
    wave.inputs["Distortion"].default_value = 0.0
    ramp = nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.interpolation = "CONSTANT"
    ramp.color_ramp.elements[1].position = 0.5
    links.new(coords.outputs["Generated"], wave.inputs["Vector"])
    links.new(wave.outputs["Fac"], ramp.inputs["Fac"])
    links.new(ramp.outputs["Color"], nodes["Background"].inputs["Color"])
    nodes["Background"].inputs["Strength"].default_value = 1.0
    return world


def build_stage(scene):
    camera = bpy.data.objects.new("camera", bpy.data.cameras.new("camera"))
    camera.data.sensor_width, camera.data.clip_start, camera.data.clip_end = SENSOR_MM, 0.01, 50
    rig = bpy.data.objects.new("rig", None)
    ground = bpy.data.objects.new("ground", bpy.data.meshes.new("ground"))
    ground.data.from_pydata([(-5, -5, 0), (5, -5, 0), (5, 5, 0), (-5, 5, 0)], [], [(0, 1, 2, 3)])
    ground.is_shadow_catcher = True
    for staged in (camera, rig, ground):
        scene.collection.objects.link(staged)
    scene.camera = camera
    lamps = []
    for light in LIGHTS:
        lamp = bpy.data.objects.new("light", bpy.data.lights.new("light", "AREA"))
        lamp.data.size, lamp.data.energy = light.size, light.watts
        lamp.parent, lamp.location = rig, light.position
        lamp.rotation_euler = Vector(light.position).to_track_quat("Z", "Y").to_euler()
        scene.collection.objects.link(lamp)
        lamps.append(lamp)
    return camera, rig, ground, lamps


def corners(objects):
    return [member.matrix_world @ Vector(corner)
            for member in objects for corner in member.bound_box]


def frame(camera, rig, shot, points):
    """Stand the camera at the shot's angles, just far enough back to frame every point."""
    azimuth, elevation = radians(shot["azimuth"]), radians(shot["elevation"])
    toward = Vector((-sin(azimuth) * cos(elevation), -cos(azimuth) * cos(elevation),
                     sin(elevation)))
    upright = Vector((0, 1, 0)) if shot["elevation"] == 90 else Vector((0, 0, 1))
    right = upright.cross(toward).normalized()
    up = toward.cross(right)
    turn = Matrix((right, up, toward)).transposed()
    seen = [turn.transposed() @ point for point in points]
    middle = [(min(p[a] for p in seen) + max(p[a] for p in seen)) / 2 for a in range(3)]
    width, height = shot["size"]
    half = [SENSOR_MM / 2 / shot["lens"] * side / max(shot["size"]) * FRAME_FILL
            for side in (width, height)]
    distance = max(max(abs(p.x - middle[0]) / half[0], abs(p.y - middle[1]) / half[1])
                   + p.z - middle[2] for p in seen)
    stand = turn @ Vector((middle[0], middle[1], middle[2] + distance))
    camera.matrix_world = Matrix.Translation(stand) @ turn.to_4x4()
    camera.data.lens = shot["lens"]
    camera.data.dof.use_dof = bool(shot.get("fstop"))
    camera.data.dof.focus_distance = distance
    if shot.get("fstop"):
        camera.data.dof.aperture_fstop = shot["fstop"]
    rig.location, rig.rotation_euler.z = turn @ Vector(middle), radians(-shot["azimuth"])


def detail_corners(detail):
    *centre, radius = detail
    return [Vector(centre) * MM + Vector((sx, sy, sz)) * radius * MM
            for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]


def main(manifest_path):
    manifest = json.loads(Path(manifest_path).read_text())
    scene = bpy.context.scene
    for leftover in list(scene.objects):
        bpy.data.objects.remove(leftover)
    set_up_renderer(scene)
    studio = studio_world(scene)
    zebra = zebra_world(scene)
    camera, rig, ground, lamps = build_stage(scene)
    objects = [import_mesh(entry) for entry in manifest["objects"]]
    out = Path(manifest["out"])
    out.mkdir(parents=True, exist_ok=True)
    wanted = set(sys.argv[sys.argv.index("--") + 2:])
    for shot in manifest["shots"]:
        if wanted and shot["name"] not in wanted:
            continue
        shown = [o for o in objects if shot["group"] in o["groups"].split(",")]
        for member in objects:
            member.hide_render = member not in shown
            finish = "zebra" if shot.get("zebra") else member["finish"]
            member.data.materials[0] = material_of(finish)
        scene.world = zebra if shot.get("zebra") else studio
        for lamp in lamps:
            lamp.hide_render = bool(shot.get("zebra"))
        bpy.context.view_layer.update()
        points = corners(shown)
        ground.location.z = min(point.z for point in points)
        ground.hide_render = bool(shot.get("zebra"))
        scene.render.film_transparent = not shot.get("zebra")
        frame(camera, rig, shot, detail_corners(shot["detail"]) if shot.get("detail") else points)
        scene.render.resolution_x, scene.render.resolution_y = shot["size"]
        scene.render.resolution_percentage = 100
        scene.render.filepath = str(out / f"{shot['name']}.png")
        bpy.ops.render.render(write_still=True)
        print("rendered", shot["name"])


if __name__ == "__main__":
    main(sys.argv[sys.argv.index("--") + 1])
