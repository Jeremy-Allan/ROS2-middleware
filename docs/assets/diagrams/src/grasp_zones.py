import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
OUT = HERE.parent
sys.path.insert(0, str(HERE))
from svgkit import Svg, FONT, LABEL, GROUP_TITLE

s = Svg(760, 300, "Grasp approach zones",
        "Side view of the approach direction of the gripper. Top grasps approach within 45 degrees of straight down. "
        "Side grasps approach from 45 degrees below horizontal to 15 degrees above. Steeper upward approaches are not used.")
cx, cy, R = 200, 150, 120

def pt(deg, r):
    a = math.radians(deg)
    return cx + r * math.cos(a), cy - r * math.sin(a)

def wedge(a0, a1, fill, stroke, extra=""):
    x0, y0 = pt(a0, R); x1, y1 = pt(a1, R)
    large = 1 if abs(a1 - a0) > 180 else 0
    # from a0 to a1 counter-clockwise on screen when a1 > a0
    s.parts.append(f'<path d="M {cx} {cy} L {x0:.1f} {y0:.1f} A {R} {R} 0 {large} 0 {x1:.1f} {y1:.1f} Z" '
                   f'fill="{fill}" stroke="{stroke}" stroke-width="1.3" {extra}/>')

wedge(-90, -45, "#dfe3f8", "#5c6bc0")          # top
wedge(-45, 15, "#d9efe8", "#3a9d82")           # side
wedge(15, 90, "#f1f2f4", "#9aa3ab", 'stroke-dasharray="4 3"')  # not used

# horizontal reference and object
s.parts.append(f'<line x1="{cx - 40}" y1="{cy}" x2="{cx + R + 28}" y2="{cy}" stroke="#90a4ae" stroke-width="1" stroke-dasharray="2 4"/>')
s.parts.append(f'<rect x="{cx - 14}" y="{cy - 14}" width="28" height="28" rx="3" fill="#ffffff" stroke="#455a64" stroke-width="1.6"/>')
s.label(cx - 30, cy + 30, "object", size=11.5, anchor="end")
s.label(cx - 46, cy - 6, "horizontal", size=11, anchor="end")
# angle labels
for deg, text in ((-45, "-45"), (15, "+15")):
    x, y = pt(deg, R + 16)
    s.label(x + (6 if deg > 0 else 4), y + 4, text + "°", size=11, anchor="start")
x, y = pt(-90, R + 14); s.label(cx, y + 6, "straight down", size=11)
# zone names inside wedges
x, y = pt(-67.5, 78); s.label(x, y + 4, "top", size=13, bold=True, color="#2f3d8f")
x, y = pt(-15, 80); s.label(x, y + 4, "side", size=13, bold=True, color="#1d6b57")
x, y = pt(52, 82); s.label(x, y + 4, "not used", size=12, color="#6b747c")

# legend on the right
lx = 400
rows = [
    ("#dfe3f8", "#5c6bc0", "Top grasp", "Approach within 45° of straight down."),
    ("#d9efe8", "#3a9d82", "Side grasp", "Approach from 45° below horizontal to 15° above."),
    ("#f1f2f4", "#9aa3ab", "Not used", "Approach more than 15° upward, from below."),
]
for i, (fill, stroke, title, text) in enumerate(rows):
    y = 76 + i * 66
    s.parts.append(f'<rect x="{lx}" y="{y}" width="22" height="22" rx="4" fill="{fill}" stroke="{stroke}" stroke-width="1.3"/>')
    s.label(lx + 34, y + 16, title, size=13.5, anchor="start", bold=True, color="#263238")
    s.label(lx + 34, y + 35, text, size=12, anchor="start")
s.label(lx, 28, "Approach direction of the gripper, side view", size=12, anchor="start", bold=True, color=GROUP_TITLE)
s.label(lx, 46, "The zones apply to every heading around the object.", size=11.5, anchor="start")
s.save(OUT / "grasp-zones.svg")
print("ok")
