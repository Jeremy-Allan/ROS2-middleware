# Optional speed controls for a single movement. 0.0 leaves it unset: Pilz
# runs at full speed, OMPL uses MoveIt's default scaling. Values are clamped
# to [0.0, 1.0] by hardware_interface_client.py before being applied.
float64 velocity_scale
float64 acceleration_scale
