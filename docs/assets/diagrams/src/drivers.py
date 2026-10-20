from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
OUT = HERE.parent
sys.path.insert(0, str(HERE))
from svgkit import Svg

# ---- Control path from the middleware to the arm -------------------------------------------
s = Svg(800, 566, "Control path from the middleware to the arm",
        "kinova_hardware_client sends planning and execution goals to move_group and gripper goals to the gripper controller. "
        "move_group sends trajectories to joint_trajectory_controller. The controllers exchange commands and states with the "
        "hardware interface, which is the Kortex driver for a real arm or the mock hardware for simulation.")

s.box(270, 14, 260, 40, "kinova_hardware_client", "ros")

s.group(40, 100, 340, 84, "MoveIt 2")
s.box(120, 134, 180, 38, "move_group", "external")

s.group(40, 236, 720, 104, "ros2_control")
s.box(95, 270, 230, 40, "joint_trajectory_controller", "ros")
s.box(455, 270, 262, 40, "gen3_lite_2f_gripper_controller", "ros")

s.group(40, 372, 720, 98, "Hardware interface")
s.box(60, 404, 350, 48, "kortex_driver/KortexMultiInterfaceHardware", "external", sub="use_fake_hardware:=false")
s.box(450, 404, 250, 48, "mock_components/GenericSystem", "external", sub="use_fake_hardware:=true")

s.box(100, 502, 250, 46, "Kinova Gen3 Lite", "robot", mono=False, sub="at robot_ip")

s.arrow([(270, 34), (210, 34), (210, 134)], label="move_action,\nexecute_trajectory", label_at=(198, 62), anchor="end")
s.arrow([(530, 34), (586, 34), (586, 270)], label="gripper_cmd", label_at=(598, 150), anchor="start")
s.arrow([(210, 172), (210, 270)], label="follow_joint_trajectory", label_at=(222, 224), anchor="start")
s.arrow([(210, 310), (210, 372)], both=True, label="position commands\nposition and velocity states", label_at=(222, 352), anchor="start")
s.arrow([(586, 310), (586, 372)], both=True)
s.arrow([(235, 452), (235, 502)], both=True, label="Kortex API\nTCP 10000, UDP 10001", label_at=(247, 480), anchor="start")
s.save(OUT / "control-path.svg")
print("ok")
