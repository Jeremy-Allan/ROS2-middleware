[Docs Home](../README.md)

# Safety and Contracts

This page describes the execution contract, the safety layers and the failure modes of the middleware. It states what the code enforces and notes where something is not enforced.

## Input contract

- Input is strictly received via ROS 2 services, there is no direct natural language entry point into this repository.
- Commands are expected to come from the LLM proxy via `/execute_recipe`. This is not enforced: `rosbridge_server` exposes every service, including `/kinova_hardware_client/joint_move` and `move_arm`, to any websocket client on port 9090.

## Execution contract

- Every task is decomposed into discrete, ordered steps.
- Each step must map to one of the ten known action types: `home`, `move_arm`, `relative_move`, `gripper`, `pickup`, `dropoff`, `pour`, `push`, `thrust`, `throw`. See [Configuration](../installation/configuration.md) for the exact contract.
- Execution is strictly sequential: `json_parser_node` runs steps one at a time and stops at the first failure. Steps are never executed in parallel.

## Status reporting

Status is reported at two levels:

- **Per-service response:** there's no status enum on the wire. A timeout or invalid request comes back as `success: false` with a message. `JointMove` and `ExecuteTrajectory` also return a MoveIt `error_code`.
- **Per-node telemetry:** each node reports one of three states, `STATE_IDLE`, `STATE_BUSY`, `STATE_FAULT` (see [Architecture](architecture.md)). There is no further state machine, such as `PENDING`, `RUNNING`, `COMPLETED` or `ABORTED`.

## Safety layers

1. **Schema validation**: the proxy validates every recipe against its JSON schema before sending it. `json_parser_node` itself only rejects unparseable JSON or an empty `steps` list up front. A step with missing parameters fails when it runs, after earlier steps have moved the arm.
2. **Action whitelist**: only the ten known action types are executable; anything else fails that step and aborts the recipe.
3. **Motion planning**: all Cartesian and joint moves go through MoveIt 2, which handles collision avoidance and feasibility checking.
4. **Execution guard**: `json_parser_node`'s `/execute_recipe` service runs on a `MutuallyExclusiveCallbackGroup`, so only one recipe executes at a time.
5. **Hardware protection**: joint limits and safety stops are enforced by the underlying Kortex driver and controllers, not by this middleware directly.

If a step fails, execution halts immediately rather than continuing partway through a plan, and the failure is visible both in the node's logs and in its telemetry status message.

## Known limitations

There is no layer that evaluates whether a schema-valid, collision-free recipe is semantically dangerous. The bridge also does not restrict which services a websocket client can call (see Input contract). MoveIt's collision checking only prevents the arm physically hitting something; it says nothing about whether a command should have been issued in the first place. This matters more than usual given the project's research focus on how LLMs respond to malicious instructions.

## Failure mode summary

| Failure Mode | Origin Layer | Cause | System Behaviour | Resolution |
|---|---|---|---|---|
| Invalid JSON recipe | JSON Parser Node | Malformed or incomplete structure | Execution rejected | Fix the JSON; validate before sending |
| Unknown action type | JSON Parser Node | Action not in the whitelist | Step fails, recipe aborts | Use one of the ten valid actions |
| Missing object mapping | Environment Mapping Node | Object key not in the object dictionary | Service returns failure | Add the object to `workspace_objects.yaml`'s `objects` |
| ROS 2 service timeout | Any node-to-node call | Node overload, planning delay, or a hung dependency | Step fails with a timeout message | Retry, or restart the affected node |
| Gripper command failure | Hardware Interface Client | Actuator limit or communication loss | Step fails | Check hardware connection, reinitialize if needed |
| Collision detected | MoveIt planning layer | Unsafe trajectory | Movement aborted | Adjust target or obstacle definitions, replan |
| Node desynchronisation | ROS 2 executor | Race condition or threading delay | Partial execution state | Restart the affected node(s) |

## System readiness definition

The system is only considered operational if all of the following hold:

- ROS 2 core is running and responsive.
- All four middleware nodes are active and reporting heartbeats.
- The MoveIt planning pipeline is operational.
- The environment mapping service responds correctly.
- `json_parser_node` can successfully execute a known-good test recipe.
- `/system/status` reports `SYSTEM_READY`, not `SYSTEM_FAULT`.

If any of these fail, treat the system as non-operational until resolved, see [Troubleshooting](../user-guide/troubleshooting.md).

Next: [Video Demos](../user-guide/demos.md)
