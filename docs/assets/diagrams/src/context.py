from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
OUT = HERE.parent
sys.path.insert(0, str(HERE))
from svgkit import Svg

s = Svg(762, 296, "System context",
        "An operator types into the terminal UI. The backend asks an LLM provider for a recipe and sends it over a websocket to the ROS 2 nodes, which plan with MoveIt 2 and drive the arm through the Kortex driver.")
W, H = 118, 46

# Row 0
s.box(18, 12, W, 34, "Operator", "external", mono=False, pill=True)
# Proxy group and row 1
s.group(10, 60, 456, 98, "embodied-ai-proxy", title_at="bottom")
s.box(18, 92, W, H, "Terminal UI", "node", mono=False, sub="instructions, status")
s.box(170, 92, W, H, "Backend", "node", mono=False, sub="prompt, validation")
s.box(322, 92, W, H, "LLM proxy", "node", mono=False, sub="provider adapters")
s.box(474, 92, W, H, "LLM provider", "external", mono=False, sub="Ollama or cloud API")
# Row 2
s.group(162, 192, 134, 92, "ROS2-middleware", title_at="bottom")
s.box(170, 208, W, H, "ROS 2 nodes", "ros", mono=False, sub="four nodes")
s.box(322, 208, W, H, "MoveIt 2", "external", mono=False, sub="planning, collisions")
s.box(474, 208, W, H, "Kortex driver", "external", mono=False, sub="ros2_control")
s.box(626, 208, W, H, "Gen3 Lite arm", "robot", mono=False, sub="arm and gripper", pill=False)

# Arrows
s.arrow([(77, 46), (77, 92)])
s.arrow([(136, 115), (170, 115)], both=True)
s.arrow([(288, 115), (322, 115)], both=True)
s.arrow([(440, 115), (474, 115)], both=True)
s.arrow([(229, 138), (229, 208)], both=True, label="websocket (rosbridge)", label_at=(240, 178), anchor="start")
s.arrow([(288, 231), (322, 231)])
s.arrow([(440, 231), (474, 231)])
s.arrow([(592, 231), (626, 231)])
s.save(OUT / "system-context.svg")
print("ok")
