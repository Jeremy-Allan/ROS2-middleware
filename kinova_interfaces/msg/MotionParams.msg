# Optional speed controls for a single movement. Leave both at 0.0 to use
# the arm's configured default (currently 10%, set in the Kinova MoveIt
# config's joint_limits.yaml). Values are clamped to [0.0, 1.0] by
# hardware_interface_client.py before being applied.
float64 velocity_scale
float64 acceleration_scale
