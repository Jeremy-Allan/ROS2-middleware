"""Top-down map of the default scene, drawn from workspace_objects.yaml.

x points up, y points to the left (base_link, seen from above). The file is read at run time, so the map
cannot drift from the data.
"""
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from svgkit import Svg, FONT, MONO, STYLES, LABEL

YAML = HERE.parents[3] / "kinova_interface" / "data" / "configs" / "env" / "workspace_objects.yaml"
if len(sys.argv) > 1:
    YAML = sys.argv[1]
OUT = sys.argv[2] if len(sys.argv) > 2 else str(HERE.parent / "workspace-map.svg")

W = H = 600
S = 373.0            # pixels per metre
CX = CY = W / 2


def px(x, y):
    """base_link (x forward, y left) to canvas pixels (x up, y left)."""
    return CX - y * S, CY - x * S


def rotation(o):
    if all(k in o for k in ("roll", "pitch", "yaw")):
        return Rotation.from_euler("xyz", [o["roll"], o["pitch"], o["yaw"]])
    return Rotation.from_quat([o["x"], o["y"], o["z"], o["w"]])


def footprint(entry):
    """Outline of the shape seen from above, as canvas points, or ('circle', cx, cy, r)."""
    pos = entry["pose"]["position"]
    rot = rotation(entry["pose"]["orientation"])
    shape = entry["shape"]
    kind, dims = shape["type"], shape["dimensions"]
    if kind == "BOX":
        sx, sy, sz = (d / 2 for d in dims)
        corners = np.array([[a * sx, b * sy, c * sz] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)])
        world = rot.apply(corners)[:, :2] + [pos["x"], pos["y"]]
        hull = ConvexHull(world)
        return "poly", [px(*world[i]) for i in hull.vertices]
    if kind == "CYLINDER":
        axis = rot.apply([0, 0, 1])
        if abs(axis[2]) > 0.99:
            cx, cy = px(pos["x"], pos["y"])
            return "circle", (cx, cy, dims[1] * S)
    raise SystemExit(f"shape {kind} is not drawn")


data = yaml.safe_load(open(YAML))
s = Svg(W, H, "Top-down map of the default scene in base_link",
        "The table, the objects and the base of the arm seen from above. x points up and y points to the left.")


def text(x, y, t, anchor="start", mono=False, size=12, color=LABEL, bold=False):
    fam = MONO if mono else FONT
    weight = ' font-weight="500"' if bold else ""
    s.parts.append(f'<text x="{x:.1f}" y="{y:.1f}" text-anchor="{anchor}" font-family="{fam}" font-size="{size}"{weight} '
                   f'fill="{color}" stroke="#ffffff" stroke-width="3.5" paint-order="stroke" stroke-linejoin="round">{t}</text>')


# grid every 0.25 m
for k in range(-3, 4):
    v = k * 0.25
    x0, y0 = px(v, -0.75)
    x1, y1 = px(v, 0.75)
    s.parts.append(f'<line x1="{x0:.1f}" y1="{y0:.1f}" x2="{x1:.1f}" y2="{y1:.1f}" stroke="#eceff4" stroke-width="1"/>')
    x0, y0 = px(-0.75, v)
    x1, y1 = px(0.75, v)
    s.parts.append(f'<line x1="{x0:.1f}" y1="{y0:.1f}" x2="{x1:.1f}" y2="{y1:.1f}" stroke="#eceff4" stroke-width="1"/>')

fill, stroke, ink = STYLES["external"]
for name, entry in data["obstacles"].items():
    kind, geo = footprint(entry)
    if kind != "poly":
        continue
    xs = [p[0] for p in geo]
    ys = [p[1] for p in geo]
    if not (0 <= min(xs) and max(xs) <= W and 0 <= min(ys) and max(ys) <= H):
        continue  # outside the map, for example a wall placed far away
    pts = " ".join(f"{x:.1f},{y:.1f}" for x, y in geo)
    s.parts.append(f'<polygon points="{pts}" fill="{fill}" fill-opacity="0.55" stroke="{stroke}" stroke-width="1.4"/>')
    text(min(xs) + 10, min(ys) + 20, name, mono=True, color=ink)

# vision workspace (default of vision.yaml)
vx0, vy0 = px(0.40, 0.40)
vx1, vy1 = px(-0.40, -0.40)
s.parts.append(f'<rect x="{vx0:.1f}" y="{vy0:.1f}" width="{vx1 - vx0:.1f}" height="{vy1 - vy0:.1f}" fill="none" '
               f'stroke="#aab2dd" stroke-width="1.4" stroke-dasharray="6 4"/>')
text(vx0 + 8, vy0 + 17, "vision workspace", size=11.5, color="#5f6b8a")

fill, stroke, ink = STYLES["ros"]
for name, entry in data["objects"].items():
    kind, geo = footprint(entry)
    if kind == "circle":
        cx, cy, r = geo
        s.parts.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r:.1f}" fill="{fill}" stroke="{stroke}" stroke-width="1.6"/>')
        left, right, top, bottom = cx - r, cx + r, cy - r, cy + r
    else:
        pts = " ".join(f"{x:.1f},{y:.1f}" for x, y in geo)
        s.parts.append(f'<polygon points="{pts}" fill="{fill}" stroke="{stroke}" stroke-width="1.6"/>')
        xs = [p[0] for p in geo]
        ys = [p[1] for p in geo]
        left, right, top, bottom = min(xs), max(xs), min(ys), max(ys)
    middle = (top + bottom) / 2 + 4
    # the label goes on the side that is free, to the left unless the object is the tray
    if name == "delivery_tray":
        text((left + right) / 2, bottom + 24, name, anchor="middle", mono=True, color=ink)
    else:
        text(left - 8, middle, name, anchor="end", mono=True, color=ink)

# base of the arm
fill, stroke, ink = STYLES["robot"]
bx, by = px(0, 0)
s.parts.append(f'<circle cx="{bx:.1f}" cy="{by:.1f}" r="9" fill="{fill}" stroke="{stroke}" stroke-width="1.6"/>')
text(bx + 15, by + 4, "base_link", mono=True, color=ink)

# axes
for (dx, dy, label) in ((0, -70, "x"), (-70, 0, "y")):
    s.parts.append(f'<path d="M {bx:.1f} {by:.1f} L {bx + dx:.1f} {by + dy:.1f}" stroke="#455a64" stroke-width="1.5" marker-end="url(#head)" fill="none"/>')
    text(bx + dx + (-4 if dx else 6), by + dy + (4 if dx else -4), label, anchor="end" if dx else "start", bold=True, size=13)

# scale bar, 0.25 m
sx0, sy0 = 28, H - 28
s.parts.append(f'<path d="M {sx0} {sy0} L {sx0 + 0.25 * S:.1f} {sy0}" stroke="#455a64" stroke-width="2"/>'
               f'<path d="M {sx0} {sy0 - 5} L {sx0} {sy0 + 5} M {sx0 + 0.25 * S:.1f} {sy0 - 5} L {sx0 + 0.25 * S:.1f} {sy0 + 5}" stroke="#455a64" stroke-width="2"/>')
text(sx0 + 0.25 * S + 10, sy0 + 4, "0.25 m", size=11.5)

s.save(OUT)
print("ok", OUT)
