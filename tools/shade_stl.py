"""Rasterize binary STL triangles to a PNG.

Pixels come from the mesh. This does not draw CAD edges and it does not invent a body
around the part. Run: uv run --with numpy --with pillow python tools/shade_stl.py
"""

from __future__ import annotations

import struct
import sys
from pathlib import Path

import numpy as np
from PIL import Image


def load_stl(path: Path) -> np.ndarray:
    """Return triangle vertices as float64 array of shape (n, 3, 3)."""
    data = path.read_bytes()
    if len(data) < 84 or data[:5].lower() == b"solid" and b"\n" in data[:80]:
        raise RuntimeError(f"not a binary STL: {path}")
    count = struct.unpack_from("<I", data, 80)[0]
    need = 84 + count * 50
    if count < 1 or len(data) < need:
        raise RuntimeError(f"truncated STL: {path}")
    dtype = np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("a", "<u2")])
    record = np.frombuffer(data, dtype=dtype, count=count, offset=84)
    return np.asarray(record["v"], dtype=np.float64)


def _camera(eye: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    forward = eye / np.linalg.norm(eye)
    right = np.cross(np.array([0.0, 0.0, 1.0]), forward)
    right /= np.linalg.norm(right)
    up = np.cross(forward, right)
    return right, up, forward


def _blur(alpha: np.ndarray, radius: int) -> np.ndarray:
    def once(image: np.ndarray) -> np.ndarray:
        padded = np.pad(image, ((radius, radius), (0, 0)))
        total = np.cumsum(padded, axis=0)
        image = (total[2 * radius :] - total[: -2 * radius]) / (2 * radius)
        padded = np.pad(image, ((0, 0), (radius, radius)))
        total = np.cumsum(padded, axis=1)
        return (total[:, 2 * radius :] - total[:, : -2 * radius]) / (2 * radius)

    soft = once(once(alpha))
    peak = soft.max()
    if peak > 0:
        soft /= peak
    return soft


def shade(
    meshes: list[tuple[np.ndarray, tuple[float, float, float]]],
    png: Path,
    width: int = 1600,
    height: int = 1000,
) -> None:
    """meshes: (triangles, rgb tint). Orthographic, lit from the upper front."""
    eye = np.array([0.55, -1.25, 0.72], dtype=np.float64)
    right, up, forward = _camera(eye)
    light = np.array([0.35, -0.45, 0.82], dtype=np.float64)
    light /= np.linalg.norm(light)
    fill = np.array([-0.8, -0.15, 0.35], dtype=np.float64)
    fill /= np.linalg.norm(fill)

    clouds = []
    for triangles, tint in meshes:
        flat = triangles.reshape(-1, 3)
        screen_x = flat @ right
        screen_y = flat @ up
        depth = flat @ forward
        points = np.stack((screen_x, screen_y, depth), axis=1).reshape(-1, 3, 3)
        clouds.append((points, triangles, np.array(tint, dtype=np.float64)))

    all_points = np.concatenate([cloud[0].reshape(-1, 3) for cloud in clouds], axis=0)
    mid = (all_points[:, :2].min(axis=0) + all_points[:, :2].max(axis=0)) / 2
    span = float(np.max(all_points[:, :2].max(axis=0) - all_points[:, :2].min(axis=0)))
    span = max(span, 1e-6)
    scale = 0.84 * min(width, height) / span

    def to_pixels(points: np.ndarray) -> np.ndarray:
        pixels = points.copy()
        pixels[:, :, 0] = (points[:, :, 0] - mid[0]) * scale + width / 2
        pixels[:, :, 1] = height / 2 - (points[:, :, 1] - mid[1]) * scale
        return pixels

    z_buffer = np.full((height, width), -np.inf, dtype=np.float64)
    image = np.zeros((height, width, 3), dtype=np.float64)
    shadow = np.zeros((height, width), dtype=np.float64)
    background = np.array([245.0, 245.0, 247.0])

    for points, triangles, _tint in clouds:
        pixels = to_pixels(points)
        _splat_shadow(pixels, shadow)

    blurred = _blur(shadow, 14)
    image[:] = background * (1.0 - 0.22 * blurred[..., None])

    for points, triangles, tint in clouds:
        pixels = to_pixels(points)
        normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        length = np.linalg.norm(normals, axis=1)
        ok = length > 1e-8
        normals[ok] /= length[ok, None]
        facing = np.clip(normals @ light, 0.0, 1.0)
        bounce = np.clip(normals @ fill, 0.0, 1.0)
        edge = np.clip((0.28 - np.abs(normals @ forward)) / 0.28, 0.0, 1.0)
        lit = 0.28 + 0.62 * facing + 0.22 * bounce
        color = np.clip(tint * lit[:, None] * (1.0 - 0.35 * edge[:, None]), 0.0, 255.0)
        _splat(pixels[ok], color[ok], z_buffer, image)

    png.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(image.astype(np.uint8), mode="RGB").save(png)


def _splat(pixels: np.ndarray, colors: np.ndarray, z_buffer: np.ndarray, image: np.ndarray) -> None:
    height, width = z_buffer.shape
    for tri, color in zip(pixels, colors, strict=True):
        x0, y0, z0 = tri[0]
        x1, y1, z1 = tri[1]
        x2, y2, z2 = tri[2]
        min_x = max(int(np.floor(min(x0, x1, x2))), 0)
        max_x = min(int(np.ceil(max(x0, x1, x2))), width - 1)
        min_y = max(int(np.floor(min(y0, y1, y2))), 0)
        max_y = min(int(np.ceil(max(y0, y1, y2))), height - 1)
        if max_x < min_x or max_y < min_y:
            continue
        xs = np.arange(min_x, max_x + 1)
        ys = np.arange(min_y, max_y + 1)
        grid_x, grid_y = np.meshgrid(xs, ys)
        denom = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
        if abs(denom) < 1e-8:
            continue
        w0 = ((y1 - y2) * (grid_x - x2) + (x2 - x1) * (grid_y - y2)) / denom
        w1 = ((y2 - y0) * (grid_x - x2) + (x0 - x2) * (grid_y - y2)) / denom
        w2 = 1.0 - w0 - w1
        depth = w0 * z0 + w1 * z1 + w2 * z2
        covered = (w0 >= 0) & (w1 >= 0) & (w2 >= 0)
        view = z_buffer[min_y : max_y + 1, min_x : max_x + 1]
        update = covered & (depth > view)
        view[update] = depth[update]
        image[min_y : max_y + 1, min_x : max_x + 1][update] = color


def _splat_shadow(pixels: np.ndarray, shadow: np.ndarray) -> None:
    """Drop each triangle straight down in screen space by a fixed offset. A blur follows."""
    dropped = pixels.copy()
    dropped[:, :, 1] += 18
    blank = np.full((shadow.shape[0], shadow.shape[1], 3), 0.0)
    fake_z = np.full_like(shadow, -np.inf)
    colors = np.ones((len(dropped), 3))
    _splat(dropped, colors, fake_z, blank)
    shadow[:] = np.maximum(shadow, blank[:, :, 0])


def main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit("usage: shade_stl.py INPUT.stl OUTPUT.png")
    shade([(load_stl(Path(sys.argv[1])), (226.0, 228.0, 232.0))], Path(sys.argv[2]))


if __name__ == "__main__":
    main()
