from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
OUT = HERE.parent
sys.path.insert(0, str(HERE))
from svgkit import Svg

# ---- Services and actions between the nodes -----------------------------------------------
s = Svg(800, 392, "Services and actions between the middleware nodes",
        "rosbridge_server calls json_parser_node. json_parser_node calls environment_mapping_node, kinova_hardware_client and MoveIt 2. "
        "kinova_hardware_client calls MoveIt 2 and the gripper controller. environment_mapping_node updates MoveIt's planning scene.")
s.box(315, 14, 150, 38, "rosbridge_server", "external")
s.box(315, 100, 150, 38, "json_parser_node", "ros")
s.box(14, 214, 206, 40, "environment_mapping_node", "ros")
s.box(430, 214, 190, 40, "kinova_hardware_client", "ros")
s.box(690, 212, 96, 44, "Gripper", "external", mono=False, sub="controller")
s.box(315, 332, 150, 40, "MoveIt 2", "external", mono=False)

s.arrow([(390, 52), (390, 100)], label="/execute_recipe\n/reset_environment", label_at=(402, 73), anchor="start")
s.arrow([(350, 138), (350, 170), (117, 170), (117, 214)], label="object and scene services", label_at=(232, 164))
s.arrow([(440, 138), (440, 170), (525, 170), (525, 214)], label="motion and gripper services", label_at=(535, 196), anchor="start")
s.arrow([(390, 138), (390, 332)], label="planning, IK, FK,\nvalidity services", label_at=(382, 262), anchor="end")
s.arrow([(117, 254), (117, 352), (315, 352)], label="/apply_planning_scene", label_at=(216, 346))
s.arrow([(525, 254), (525, 352), (465, 352)], label="move_action,\nexecute_trajectory", label_at=(535, 300), anchor="start")
s.arrow([(620, 234), (690, 234)], label="gripper_cmd", label_at=(655, 225))
s.save(OUT / "node-services.svg")

# ---- Status and fault flow ------------------------------------------------------------------
t = Svg(800, 300, "Status and fault flow",
        "Three nodes publish heartbeats on /status/node_report. telemetry_node publishes /system/status and talks to the fault controller.")
t.box(14, 14, 206, 38, "json_parser_node", "ros")
t.box(14, 74, 206, 38, "environment_mapping_node", "ros")
t.box(14, 134, 206, 38, "kinova_hardware_client", "ros")
t.box(14, 236, 206, 38, "fault_controller", "external")
t.box(380, 76, 150, 44, "telemetry_node", "ros")
t.box(640, 76, 146, 44, "rosbridge_server", "external")

for y in (33, 93, 153):
    t.arrow([(220, y), (300, y), (300, 98), (380, 98)] if y != 93 else [(220, y), (300, y), (300, 98), (380, 98)])
t.label(308, 62, "/status/node_report", anchor="start")
t.arrow([(530, 98), (640, 98)], label="/system/status", label_at=(585, 90))
t.arrow([(117, 236), (117, 172)], label="/fault_controller/is_faulted", label_at=(127, 208), anchor="start")
t.arrow([(455, 120), (455, 255), (220, 255)], both=True, label="reset_fault, configure and\nactivate controller", label_at=(465, 205), anchor="start")
t.save(OUT / "status-flow.svg")
print("ok")
