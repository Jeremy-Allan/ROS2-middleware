# TF/URDF frame, joint and link names for the Kinova Gen3 Lite.
BASE_FRAME = 'base_link'
TOOL_FRAME = 'tool_frame'

# The arm's six revolute joints, in chain order (base -> wrist).
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# Links allowed to touch an object once it's attached to the gripper, so the
# fingers actually closing around it doesn't register as a collision.
GRIPPER_TOUCH_LINKS = [
    TOOL_FRAME, "gripper_base_link",
    "left_finger_dist_link", "left_finger_prox_link",
    "right_finger_dist_link", "right_finger_prox_link",
]

# Gripper commands (rad). Closing stops on the object (allow_stalling), so one closed value fits all.
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 0.8
# Gap between the finger pads when fully open (m).
GRIPPER_MAX_OPENING = 0.109
# How far the fingertips reach past the tool_frame origin (pad centre), roughly.
FINGERTIP_LENGTH = 0.03
