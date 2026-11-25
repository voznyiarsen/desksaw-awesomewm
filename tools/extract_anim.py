#!/usr/bin/env python3
"""Extract the desksaw skeleton animation clips from sawianBase.tscn into Lua."""
import re, math, os

TSCN = "/home/tsuchinoko/desksaw/scenes/sawianBase.tscn"
OUT = "/home/tsuchinoko/.config/awesome/desksaw/anim.lua"

s = open(TSCN).read()

# --- collect Animation sub-resources ---
clips = {}
for m in re.finditer(r'\[sub_resource type="Animation" id="([^"]+)"\]\n(.*?)(?=\n\[sub_resource|\n\[node)', s, re.S):
    aid, body = m.groups()
    name = re.search(r'resource_name = "([^"]*)"', body)
    name = name.group(1) if name else "RESET"
    # Godot writes whole seconds as "0" or "2" (no decimal point), so the
    # pattern must accept integers too -- otherwise idleagain/attack/dance/
    # oldIdle silently get length 0 and their pose freezes.
    length = re.search(r'length = ([\d.eE+-]+)', body)
    length = float(length.group(1)) if length else 0.0
    loop = 1 if re.search(r'loop_mode = 1', body) else 0

    tracks = {}
    # each track block
    for tm in re.finditer(
            r'tracks/(\d+)/path = NodePath\("([^"]+)"\)\n'
            r'(?:tracks/\d+/interp = \d+\n)?'
            r'(?:tracks/\d+/loop_wrap = \w+\n)?'
            r'tracks/\d+/keys = \{\n'
            r'"times": PackedFloat32Array\(([^)]*)\)[^\n]*\n'
            r'"transitions": PackedFloat32Array\(([^)]*)\)[^\n]*\n'
            r'"update": \d+,\n'
            r'"values": \[(.*?)\]\n\}', body, re.S):
        idx, path, times, trans, values = tm.groups()
        times = [float(x) for x in times.split(",") if x.strip()]
        vals = values.strip()
        parsed = []
        if "Vector2(" in vals:
            for v in re.split(r',\s*(?=Vector2\()', vals):
                v = v.strip().rstrip(",")
                vm = re.match(r'Vector2\(([^,]+),\s*([^)]+)\)', v)
                if vm:
                    parsed.append(("v2", float(vm.group(1)), float(vm.group(2))))
        else:
            for v in vals.replace("\n", " ").split(","):
                v = v.strip()
                if not v:
                    continue
                try:
                    parsed.append(("f", float(v)))
                except ValueError:
                    pass
        # short path within the skeleton
        short = path
        for pref in ("RigidBodies/skeleton/Skeleton2D/", "RigidBodies/"):
            if short.startswith(pref):
                short = short[len(pref):]
        short = short.replace("hip/IKNodes/", "ik/")
        tracks[short] = {"times": times, "values": parsed, "trans": [float(x) for x in trans.split(",") if x.strip()]}

    # Godot defaults an Animation's length to 1.0 when the scene omits the
    # `length` line entirely (idleagain, attack, dance, oldIdle). Derive it
    # from the furthest keyframe instead of leaving it 0, which would freeze
    # the clip's pose.
    if not length or length <= 0:
        for _, t in tracks.items():
            times = t["times"]
            if len(times) > 0:
                last = times[-1]
                if last > length:
                    length = last
        if length <= 0:
            length = 1.0

    clips[name] = {"length": length, "loop": loop, "tracks": tracks}

def fmt_num(x):
    if abs(x - round(x)) < 1e-9:
        return str(int(round(x)))
    return f"{x:.6g}"

lines = []
lines.append("-- anim.lua")
lines.append("-- desksaw.anim -- generated from scenes/sawianBase.tscn by tools/extract_anim.py")
lines.append("-- Keyframed skeleton poses (hip + IK targets + tail), sampled from the Godot scene.")
lines.append("-- Do not edit by hand: re-run the generator instead.")
lines.append("return {")
for name, c in clips.items():
    if name == "RESET" or not c["tracks"]:
        continue
    lines.append(f'  {name} = {{')
    lines.append(f'    length = {fmt_num(c["length"])}, loop = {"true" if c["loop"] else "false"},')
    lines.append("    tracks = {")
    for tname, t in c["tracks"].items():
        times = ", ".join(fmt_num(x) for x in t["times"])
        if t["values"] and t["values"][0][0] == "v2":
            vals = ", ".join("{" + fmt_num(v[1]) + "," + fmt_num(v[2]) + "}" for v in t["values"])
        else:
            vals = ", ".join(fmt_num(v[1]) for v in t["values"])
        lines.append(f'      ["{tname}"] = {{ {{{times}}}, {{{vals}}} }},')
    lines.append("    },")
    lines.append("  },")
lines.append("}")

os.makedirs(os.path.dirname(OUT), exist_ok=True)
open(OUT, "w").write("\n".join(lines) + "\n")
print(f"wrote {OUT}")
for name, c in clips.items():
    if name != "RESET" and c["tracks"]:
        print(f"  {name}: len={c['length']} loop={c['loop']} tracks={len(c['tracks'])}")
