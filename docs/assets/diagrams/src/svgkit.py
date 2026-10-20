"""Tiny SVG diagram kit for the manual's architecture figures.

Boxes, groups and arrows are placed by hand with explicit coordinates so the layout is
exactly what is intended. Output is a self-contained SVG with a white card background, so it
reads the same in the light and dark site themes.
"""
from xml.sax.saxutils import escape

FONT = "Roboto, 'Helvetica Neue', Arial, sans-serif"
MONO = "'Roboto Mono', 'SFMono-Regular', Consolas, monospace"

STYLES = {
    # fill, stroke, text
    "node": ("#eef0fb", "#5c6bc0", "#1a1f36"),
    "external": ("#f3f4f6", "#78909c", "#263238"),
    "robot": ("#fff4e5", "#ef9a3c", "#3e2a05"),
    "ros": ("#e8f5f1", "#3a9d82", "#10332a"),
}
ARROW = "#455a64"
LABEL = "#37474f"
GROUP_FILL, GROUP_STROKE, GROUP_TITLE = "#fafbff", "#aab2dd", "#5f6b8a"


class Svg:
    def __init__(self, width, height, title="", desc=""):
        self.w, self.h = width, height
        self.parts = []
        self.title, self.desc = title, desc

    # -- primitives --------------------------------------------------------
    def group(self, x, y, w, h, title, title_at="top"):
        ty = y + 18 if title_at == "top" else y + h - 9
        self.parts.append(
            f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="{GROUP_FILL}" '
            f'stroke="{GROUP_STROKE}" stroke-width="1.2" stroke-dasharray="5 4"/>'
            f'<text x="{x + 12}" y="{ty}" font-family="{FONT}" font-size="12" font-weight="500" '
            f'fill="{GROUP_TITLE}" letter-spacing="0.3">{escape(title)}</text>')

    def box(self, x, y, w, h, text, kind="node", mono=True, sub=None, pill=False):
        fill, stroke, ink = STYLES[kind]
        rx = h / 2 if pill else 5
        self.parts.append(
            f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" fill="{fill}" stroke="{stroke}" stroke-width="1.4"/>')
        family, size = (MONO, 12.5) if mono else (FONT, 13.5)
        cx = x + w / 2
        if sub:
            self.parts.append(f'<text x="{cx}" y="{y + h / 2 - 3}" text-anchor="middle" font-family="{family}" '
                              f'font-size="{size}" fill="{ink}">{escape(text)}</text>')
            self.parts.append(f'<text x="{cx}" y="{y + h / 2 + 13}" text-anchor="middle" font-family="{FONT}" '
                              f'font-size="11" fill="{LABEL}">{escape(sub)}</text>')
        else:
            self.parts.append(f'<text x="{cx}" y="{y + h / 2 + 4.5}" text-anchor="middle" font-family="{family}" '
                              f'font-size="{size}" fill="{ink}">{escape(text)}</text>')

    def arrow(self, pts, label=None, label_at=None, dashed=False, both=False, anchor="middle", color=ARROW):
        d = "M " + " L ".join(f"{px} {py}" for px, py in pts)
        dash = ' stroke-dasharray="5 4"' if dashed else ""
        start = ' marker-start="url(#head)"' if both else ""
        self.parts.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="1.5"{dash} '
                          f'marker-end="url(#head)"{start} stroke-linejoin="round"/>')
        if label:
            lx, ly = label_at
            for i, line in enumerate(label.split("\n")):
                self.parts.append(
                    f'<text x="{lx}" y="{ly + i * 13.5}" text-anchor="{anchor}" font-family="{FONT}" font-size="11.5" '
                    f'fill="{LABEL}" stroke="#ffffff" stroke-width="3.5" paint-order="stroke" '
                    f'stroke-linejoin="round">{escape(line)}</text>')

    def label(self, x, y, text, size=11.5, anchor="middle", bold=False, color=LABEL):
        weight = ' font-weight="500"' if bold else ""
        self.parts.append(f'<text x="{x}" y="{y}" text-anchor="{anchor}" font-family="{FONT}" font-size="{size}"{weight} '
                          f'fill="{color}">{escape(text)}</text>')

    # -- output ------------------------------------------------------------
    def render(self):
        defs = (
            f'<defs><marker id="head" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6.5" markerHeight="6.5" orient="auto-start-reverse">'
            f'<path d="M0 0 L10 5 L0 10 z" fill="{ARROW}"/></marker>'
            f'</defs>')
        head = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {self.w} {self.h}" width="{self.w}" height="{self.h}" '
                f'role="img" aria-label="{escape(self.title)}">')
        meta = f'<title>{escape(self.title)}</title>' + (f'<desc>{escape(self.desc)}</desc>' if self.desc else "")
        bg = (f'<rect x="0.5" y="0.5" width="{self.w - 1}" height="{self.h - 1}" rx="8" fill="#ffffff" stroke="#dfe2ea"/>')
        return head + meta + defs + bg + "".join(self.parts) + "</svg>\n"

    def save(self, path):
        with open(path, "w") as f:
            f.write(self.render())
