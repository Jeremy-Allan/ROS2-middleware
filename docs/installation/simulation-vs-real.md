# Simulation vs real arm

`robot.launch.py` starts the same nodes for a simulated and for a real arm. The argument `use_fake_hardware` selects the hardware interface of `ros2_control`.

## Start

Simulation, the default:

```bash
ros2 launch kinova_interface robot.launch.py
```

Real arm:

```bash
ros2 launch kinova_interface robot.launch.py use_fake_hardware:=false robot_ip:=192.168.1.10
```

With `use_fake_hardware:=false`, the launch stops with an error when `robot_ip:=` is not on the command line. The default address is not used.

## Differences

| | Simulation | Real arm |
|---|---|---|
| Hardware interface | `mock_components/GenericSystem` | `kortex_driver/KortexMultiInterfaceHardware` |
| Joint states | Follow the commands. | Come from the arm. |
| Fault controller | Not started. `/system/status` does not report a hardware fault. | Started. A fault of the arm sets `/system/status` to `SYSTEM_FAULT`. |
| Gripper | Reaches the commanded position. There is no contact with objects. | Stops when the fingers meet an object. |
| Objects | Entries of the planning scene. `throw` moves the object along a computed arc and `pour` has no liquid. | Entries of the planning scene. The real objects must be where `workspace_objects.yaml` puts them. |
| Camera | Not simulated. | Kinect v2. |

## Before the first run on the real arm

1. Run the recipe in simulation.
2. Compare `workspace_objects.yaml` with the table. Check the positions, the dimensions and the height of the table.
3. Remove everything from the workspace that is not in the file. The planner does not know about other objects. See [Safety and contracts](../technical/safety-and-contracts.md).
4. Set a `speed` below 1.0 in the steps that take one.

To clear a fault of the arm, see [Fault handling](../technical/safety-and-contracts.md#fault-handling).
