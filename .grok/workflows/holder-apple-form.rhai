let meta = #{
    name: "holder-apple-form",
    description: "Turn the monitor clamp into one checked assembly, then a clean agent accepts or sends it back",
    when_to_use: "Redesign monitor-stand-extenstions toward the macropad and camx fastener discipline. Loops up to three times. Does not push.",
    phases: [
        #{ title: "Brainstorm", detail: "three mechanical moves" },
        #{ title: "Review", detail: "one brief that keeps the interfaces" },
        #{ title: "Implement", detail: "edit, test, export, shade" },
        #{ title: "Critique", detail: "a clean agent judges the solids" },
        #{ title: "Commit", detail: "commit on main only if accepted" },
    ],
};

let idea_schema = #{
    "type": "object",
    "required": ["move", "files", "risk"],
    "properties": #{
        "move": #{ "type": "string" },
        "files": #{ "type": "string" },
        "risk": #{ "type": "string" },
    },
};
let brief_schema = #{
    "type": "object",
    "required": ["brief", "reject"],
    "properties": #{
        "brief": #{ "type": "string" },
        "reject": #{ "type": "string" },
    },
};
let impl_schema = #{
    "type": "object",
    "required": ["ok", "evidence"],
    "properties": #{
        "ok": #{ "type": "boolean" },
        "evidence": #{ "type": "string" },
    },
};
let note_schema = #{
    "type": "object",
    "required": ["accept", "done", "stop_adding", "critique"],
    "properties": #{
        "accept": #{ "type": "boolean" },
        "done": #{ "type": "boolean" },
        "stop_adding": #{ "type": "boolean" },
        "critique": #{ "type": "string" },
    },
};
let commit_schema = #{
    "type": "object",
    "required": ["committed", "sha", "subject"],
    "properties": #{
        "committed": #{ "type": "boolean" },
        "sha": #{ "type": "string" },
        "subject": #{ "type": "string" },
    },
};

let prior = "The published solids are rounded boxes. Make one clamp assembly whose screws, nuts, and clearances come from the numbers already in the repo.";
if args != () && type_of(args) == "map" && args.critique != () && args.critique != "" {
    prior = args.critique;
}

let root = "/home/worxbend/Projects/monitor-stand-extenstions";
let cad_ref = "/home/worxbend/Projects/monitor-stand-extenstions-holder-cad/src/holder/band.py";
let macro = "/home/worxbend/Projects/macropad";
let camx = "/home/worxbend/Projects/camx-pro/hardware/tools/verify_mechanical_interfaces.py";

fn map_ok(result) {
    if result == () { return false; }
    if result.success != true { return false; }
    if type_of(result.output) != "map" { return false; }
    return true;
}

let rounds = [0, 1, 2];
let keep = true;
let accept = false;
let done = false;
let stop_adding = false;
let critique = prior;
let evidence = "no implement pass yet";

for round in rounds {
    if keep {
        phase("Brainstorm");
        let axes = [
            "one bracket silhouette: cylinders or 45 degree chamfers on non-mating edges, flat mouths where a nut slot exits",
            "screw stacks: each bolt length equals head plus the parts it clamps plus nut engagement, taken from params.py",
            "final assembly: band halves clear the column, block bolts to the boss, one arm, camera plate on the last 1/4 inch face",
        ];
        let jobs = [];
        let n = 0;
        for axis in axes {
            let bp = "Read before you answer. Repo: ";
            bp += root;
            bp += "/src/holder/params.py, band.py, block.py, accessories.py, README.md, docs/renders/hero.png. ";
            bp += "Fit reference, do not copy the whole branch: ";
            bp += cad_ref;
            bp += ". Form reference, do not copy the keyboard: ";
            bp += macro;
            bp += "/cad/README.md and docs/images/cad/hero_aluminium.png if present. ";
            bp += "Fastener reference: ";
            bp += camx;
            bp += ". Use read_file. Do not edit. One move only for: ";
            bp += axis;
            bp += ". This product is a monitor-column clamp and a camera shelf. Do not add a keypad, a display, a PCB, printed GoPro prongs, or a phone cradle. ";
            bp += "Do not change column_w, column_d, PHONE_EAR_W, the ZV-E10 slot, or the GoPro 13 by 5 pocket. column_r stays 12 until a coupon is printed. ";
            bp += "Prior critique, JSON: ";
            bp += json_encode(prior);
            jobs.push(#{
                prompt: bp,
                label: "brainstorm:" + n.to_string(),
                capability_mode: "read-only",
                output_schema: idea_schema,
            });
            n += 1;
        }
        let ideas = parallel(jobs);
        let packed = "[]";
        if ideas != () && type_of(ideas) == "array" {
            let rows = [];
            for idea in ideas {
                if map_ok(idea) {
                    rows.push(idea.output);
                }
            }
            if rows.len() > 0 {
                packed = json_encode(rows);
            }
        }

        phase("Review");
        let rp = "Read ";
        rp += root;
        rp += "/src/holder/params.py and block.py before you answer. Pick one buildable brief from: ";
        rp += packed;
        rp += ". Reject a keypad, electronics, printed GoPro fingers, a phone cradle, a new ball-head solid, and any edit to column_w, column_d, PHONE_EAR_W, the ZV-E10 slot length, or the GoPro pocket size. ";
        rp += "The brief must keep captive nuts in side slots, 1/4-20 and M3 holes where they are, and hinge clearance. ";
        rp += "Nut slots need a flat mouth, so do not raise the block plate or riser corner past 3 mm, or the arm corner past 5 mm, unless the brief also removes the seam fillet and proves the bounding box. ";
        rp += "Order the band ribs to clear the column by the TPU fit, as in the holder-cad band, and to update the test that currently demands a rib intersection. ";
        rp += "Prior critique, JSON: ";
        rp += json_encode(prior);
        rp += ". Return brief and reject.";
        let pick = agent(rp, #{
            label: "review:brief",
            capability_mode: "read-only",
            output_schema: brief_schema,
        });
        let brief = "Clear the band ribs by the TPU fit. Recess screw heads. Shade the assembly from STL. Do not invent equipment sizes.";
        if map_ok(pick) && pick.output.brief != () && pick.output.brief != "" {
            brief = pick.output.brief;
        }
        log("brief " + round.to_string());

        phase("Implement");
        let ip = "Edit only ";
        ip += root;
        ip += ". Read params.py, band.py, block.py, accessories.py, tests, tools/render_gallery.py, and ";
        ip += cad_ref;
        ip += " first. Port the column clearance from that band. Do not reset main onto holder-cad. Do not delete the block or the accessories. ";
        ip += "Keep one valid solid per part. Captive nuts stay in side slots. Phone stays a pocket. Do not print GoPro prongs. Do not change PHONE_EAR_W, column_w, column_d, column_r, the ZV-E10 slot, or the GoPro pocket size. ";
        ip += "Coupon measuring faces stay sharp. Do not commit coupon mesh noise. ";
        ip += "A seam fillet that throws or inflates the block bounding box must be reverted. Plate and riser corners stay at 3 mm and the arm at 5 mm unless you prove the print bounding box. ";
        ip += "Derive screw lengths from the stack in params.py. Update README hardware lines to those lengths. ";
        ip += "Run uv run ruff check src tests tools, uv run pytest -q, and uv run python -m holder. All must exit 0 or ok is false. ";
        ip += "Then uv run --with numpy --with pillow python tools/render_gallery.py. Shade from STL. Do not call an image generator. ";
        ip += "Do not commit. Do not push. Do not edit .grok. ";
        ip += "Brief: ";
        ip += brief;
        let built = agent(ip, #{
            label: "implement:holder",
            capability_mode: "all",
            output_schema: impl_schema,
        });
        let impl_ok = false;
        evidence = "implement agent returned nothing";
        if map_ok(built) {
            if built.output.ok == true { impl_ok = true; }
            if built.output.evidence != () { evidence = built.output.evidence; }
        }

        phase("Critique");
        let cp = "You are a clean reviewer. You did not write this pass. Work in ";
        cp += root;
        cp += ". Read git diff, docs/renders/hero.png, docs/renders/arm.png, src/holder/band.py, and README.md. ";
        cp += "You may run uv run pytest -q. Do not edit. Do not commit. ";
        cp += "accept is true only when pytest would pass, the band ribs clear the column, screw lengths in the README match params.py, and the hero is an STL shade of the clamp plus an arm plus the camera plate. ";
        cp += "accept is false if the diff adds a keypad, a PCB, printed GoPro prongs, a phone cradle, or a changed PHONE_EAR_W, column size, ZV-E10 slot, or GoPro pocket. ";
        cp += "done is true only when the hero reads as one bracket with recessed fasteners, not a stack of square pads. ";
        cp += "stop_adding is true when another feature would add a part the clamp does not need. ";
        cp += "critique is the single next fix, or says no further change helps. ";
        cp += "Implement evidence, JSON: ";
        cp += json_encode(evidence);
        let note = agent(cp, #{
            label: "review:clean",
            capability_mode: "execute",
            output_schema: note_schema,
        });
        accept = false;
        done = false;
        stop_adding = false;
        if map_ok(note) {
            if note.output.accept == true { accept = true; }
            if note.output.done == true { done = true; }
            if note.output.stop_adding == true { stop_adding = true; }
            if note.output.critique != () && note.output.critique != "" {
                critique = note.output.critique;
                prior = note.output.critique;
            }
        }
        if impl_ok != true { accept = false; }
        if accept == true && done == true { keep = false; }
        log("round " + round.to_string() + " accept=" + accept.to_string() + " done=" + done.to_string());
    }
}

phase("Commit");
let gate = "REJECT";
if accept == true { gate = "ACCEPT"; }
let gp = "Work in ";
gp += root;
gp += " on branch main. Gate word: ";
gp += gate;
gp += ". If the gate is not ACCEPT, do not commit. Leave the working tree as it is. ";
gp += "If the gate is ACCEPT: run uv run pytest -q. If it fails, do not commit. ";
gp += "Confirm git branch --show-current prints main. If it does not, do not checkout and do not commit. ";
gp += "Stage only paths this pass changed under src/, tests/, tools/, README.md, docs/, and exports/. ";
gp += "Do not stage coupon files unless src/holder/coupon.py changed. Do not stage .grok. ";
gp += "Commit with git commit -F and a message file under /tmp. Subject: feat(assembly): then why the clamp became one checked assembly. ";
gp += "Do not push. Do not amend. Do not reset. Do not run smart_commit.sh. ";
gp += "Return committed, sha, and subject. Empty sha if you did not commit. Critique, JSON: ";
gp += json_encode(critique);
let saved = agent(gp, #{
    label: "commit:main",
    capability_mode: "all",
    output_schema: commit_schema,
});

let sha = "";
let subject = "";
let committed = false;
if map_ok(saved) {
    if saved.output.committed == true { committed = true; }
    if saved.output.sha != () { sha = saved.output.sha; }
    if saved.output.subject != () { subject = saved.output.subject; }
}
log(gate + " committed=" + committed.to_string() + " " + sha);
write_scratch_file("handoff.md", critique);
complete(#{
    summary: critique,
    accept: accept,
    done: done,
    stop_adding: stop_adding,
    committed: committed,
    sha: sha,
    subject: subject,
});
