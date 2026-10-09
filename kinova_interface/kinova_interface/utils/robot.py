from moveit_msgs.msg import MoveItErrorCodes

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

# The gripper's driven joint, the other finger joints mimic it
GRIPPER_JOINT = 'right_finger_bottom_joint'
# Gripper commands (rad). Closing stops on the object (allow_stalling), so one closed value fits all.
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 0.8
# Gap between the finger pads when fully open (m).
GRIPPER_MAX_OPENING = 0.109
# How far the fingertips reach past the tool_frame origin (pad centre), roughly.
FINGERTIP_LENGTH = 0.03


# Every MoveIt error code, readable, for logs and failure messages
MOVEIT_ERRORS = {
    MoveItErrorCodes.FAILURE: 'MoveIt failed',
    MoveItErrorCodes.PLANNING_FAILED: 'planning failed (path blocked by an obstacle or self-collision)',
    MoveItErrorCodes.INVALID_MOTION_PLAN: 'invalid motion plan',
    MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: 'motion plan invalidated by an environment change',
    MoveItErrorCodes.CONTROL_FAILED: 'control failed during execution (hardware error)',
    MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA: 'unable to acquire sensor data',
    MoveItErrorCodes.TIMED_OUT: 'timed out',
    MoveItErrorCodes.PREEMPTED: 'preempted',
    MoveItErrorCodes.START_STATE_IN_COLLISION: 'start state in collision (robot is currently in collision)',
    MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS: 'start state violates path constraints',
    MoveItErrorCodes.GOAL_IN_COLLISION: 'goal in collision (target inside an obstacle)',
    MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: 'goal violates path constraints',
    MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: 'goal constraints violated',
    MoveItErrorCodes.INVALID_GROUP_NAME: 'invalid group name',
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: 'invalid goal constraints',
    MoveItErrorCodes.INVALID_ROBOT_STATE: 'invalid robot state',
    MoveItErrorCodes.INVALID_LINK_NAME: 'invalid link name',
    MoveItErrorCodes.INVALID_OBJECT_NAME: 'invalid object name',
    MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: 'frame transform failed',
    MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE: 'collision checking unavailable',
    MoveItErrorCodes.ROBOT_STATE_STALE: 'robot state stale',
    MoveItErrorCodes.SENSOR_INFO_STALE: 'sensor info stale',
    MoveItErrorCodes.COMMUNICATION_FAILURE: 'communication failure',
    MoveItErrorCodes.START_STATE_INVALID: 'start state invalid',
    MoveItErrorCodes.GOAL_STATE_INVALID: 'goal state invalid',
    MoveItErrorCodes.UNRECOGNIZED_GOAL_TYPE: 'unrecognized goal type',
    MoveItErrorCodes.CRASH: 'MoveIt crashed',
    MoveItErrorCodes.ABORT: 'aborted by MoveIt',
    MoveItErrorCodes.NO_IK_SOLUTION: 'out of reach (no IK solution)',
}


# MoveIt failures that happen before the arm moves, so it's safe to try again
BEFORE_MOTION_ERRORS = {
    MoveItErrorCodes.FAILURE,
    MoveItErrorCodes.PLANNING_FAILED,
    MoveItErrorCodes.INVALID_MOTION_PLAN,
    MoveItErrorCodes.NO_IK_SOLUTION,
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS,
}


def moveit_error(code):
    return MOVEIT_ERRORS.get(code, f'unknown MoveIt error code {code}')
