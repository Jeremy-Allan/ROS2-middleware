# TF/URDF frame names for the Kinova Gen3 Lite, shared across arm_actions.py,
# hardware_interface_client.py, and environment_mapping_node.py so a frame
# rename can't silently miss one of them.
BASE_FRAME = 'base_link'
TOOL_FRAME = 'tool_frame'
