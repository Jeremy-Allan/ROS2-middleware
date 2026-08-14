[Docs Home](README.md)

# Safety and Contracts

This page describes the execution contract, safety layers, and failure model this middleware is designed against. It's a mix of what's genuinely enforced in code today and some forward-looking design intent, both are marked as such below.

## Input contract

- Input is strictly received via ROS 2 services, there is no direct natural language entry point into this repository.
- All commands must originate from a validated upstream system (in practice, the LLM proxy, via `/execute_recipe`).

## Execution contract

- Every task is decomposed into discrete, ordered steps.
- Each step must map to one of the four known action types: `home`, `move_arm`, `relative_move`, `gripper`. See [Configuration](configuration.md) for the exact contract and a note on where this repo's older docs disagreed with the actual code.
- Execution is strictly sequential: `json_parser_node` runs steps one at a time and stops at the first failure. There is no parallel step execution today.

## What's actually implemented vs design intent

The system's real, currently-implemented state model is simpler than some earlier project documentation described:

- **Per-service response:** every custom service returns just `success` (bool) and `message` (string). There's no separate `TIMEOUT` or `INVALID_REQUEST` enum on the wire, a timeout or invalid request just comes back as `success: false` with an explanatory message.
- **Per-node telemetry:** each node reports one of three states, `STATE_IDLE`, `STATE_BUSY`, `STATE_FAULT` (see [Architecture](architecture.md)). There isn't a richer `PENDING` / `RUNNING` / `COMPLETED` / `ABORTED` state machine implemented anywhere in code today, even though earlier design notes described one. If that richer model becomes real, this page should be updated to match.

## Safety layers

1. **Schema validation**: malformed or incomplete recipe JSON is rejected before execution starts (`json_parser_node`'s `execute_recipe`, and independently, the proxy's Pydantic schema before a recipe is even sent).
2. **Action whitelist**: only the four known action types are executable; anything else fails that step and aborts the recipe.
3. **Motion planning**: all Cartesian and joint moves go through MoveIt 2, which handles collision avoidance and feasibility checking. No raw joint commands are accepted from outside this layer.
4. **Execution guard**: `json_parser_node`'s `/execute_recipe` service runs on a `MutuallyExclusiveCallbackGroup`, so only one recipe executes at a time.
5. **Hardware protection**: joint limits and safety stops are enforced by the underlying Kortex driver and controllers, not by this middleware directly.

If a step fails, execution halts immediately rather than continuing partway through a plan, and the failure is visible both in the node's logs and in its telemetry status message.

## Known gap relevant to safety

There is currently no layer that evaluates whether a schema-valid, collision-free recipe is semantically dangerous. MoveIt's collision checking only prevents the arm physically hitting something; it says nothing about whether a command should have been issued in the first place. See the note in [Configuration](configuration.md) about the proxy's undefined `safety_violation` handling, this matters more than usual given the project's research focus on how LLMs respond to malicious instructions.

## Failure mode summary

| Failure Mode | Origin Layer | Cause | System Behaviour | Resolution |
|---|---|---|---|---|
| Invalid JSON recipe | JSON Parser Node | Malformed or incomplete structure | Execution rejected | Fix the JSON; validate before sending |
| Unknown action type | JSON Parser Node | Action not in the whitelist | Step fails, recipe aborts | Use one of the four valid actions |
| Missing object mapping | Environment Mapping Node | Object key not in the coordinate dictionary | Service returns failure | Add the object to `coordinate_dictionary.json` |
| ROS 2 service timeout | Any node-to-node call | Node overload, planning delay, or a hung dependency | Step fails or the call hangs (see the no-timeout gap noted in Architecture) | Retry, or restart the affected node |
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

If any of these fail, treat the system as non-operational until resolved, see [Troubleshooting](troubleshooting.md).

Next: [Video Demos](demos.md)
