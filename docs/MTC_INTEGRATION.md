# MoveIt Task Constructor Integration

## Status and scope

This document is the implementation and commissioning contract for staged
pick-and-place on the Kinova Gen3 Lite. It complements the repository README;
where the two disagree, this document describes the MTC path.

The existing middleware remains the Python orchestration layer. A
`PickPlace` ROS 2 action is the boundary between recipes and the MTC task
executor. This boundary is intentional: the official ROS 2 MTC tutorial and
the supported Humble MTC core API are C++-first. A Python task executor is
acceptable only when a matching, tested MTC Python binding is present in the
same Humble/MoveIt overlay. Sending a sequence of ordinary `MoveGroup` goals
from Python is not MoveIt Task Constructor and must not be presented as such.

The Python calls in this implementation were checked against MTC's `humble`
branch at commit `756634951326ae17ae099882f7110c6f1d0a98c0`. Record and test
the exact commit or Debian build used in the robot workspace; a package
version string alone does not prove that all required Python bindings are
present.

**This Windows checkout cannot run or validate ROS 2 Humble, MoveIt 2,
MoveIt Task Constructor, `ros2_control`, or the Kinova driver.** The
ROS-independent configuration tests can run here, but all ROS build,
planning, fake-hardware, and physical-hardware gates in this guide must be run
on Ubuntu 22.04 with ROS 2 Humble.

Reference tutorial:
<https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html>

## Why the previous motion was unreliable

The legacy pick sequence sends unrelated commands for:

1. moving the arm to an XYZ point;
2. closing the gripper;
3. moving upward;
4. moving to the destination; and
5. opening the gripper.

The arm goal constrains only the position of `tool_frame` inside a small
sphere. It does not constrain tool orientation or the path used for the final
approach. The sampling planner can choose different IK solutions and different
paths between runs. The subsequent lift is another independent sampled plan,
so the grasp posture found by the first plan is not part of a coherent
manipulation solution.

The legacy flow also does not:

- model the item as a collision object;
- allow hand/object contact only during grasp;
- attach the item to `tool_frame`;
- carry the attached object through transfer planning;
- detach it at the destination;
- enforce Cartesian approach, lift, lower, and retreat segments; or
- rank complete end-to-end task solutions.

MTC fixes the task-composition problem. It does not by itself fix inaccurate
object poses, an incorrect TCP transform, unsafe gripper calibration, or a
misconfigured robot model. Those are explicit commissioning gates below.

## Architecture

The intended control flow is:

```text
recipe or external caller
  -> PickPlace action client in the Python middleware
  -> one serialized PickPlace goal
  -> manipulation_objects.json validation and ID resolution
  -> MTC task construction
  -> plan one or more complete task solutions
  -> publish the selected solution for RViz introspection
  -> execute only when plan_only is false
  -> return the exact failed stage and MoveIt error
```

The legacy `move_arm`, `relative_move`, home, and gripper services remain for
supervised maintenance, but automated recipes reject them. Recipes accept
only atomic `pick_and_place` steps and always request execution. The model
chooses object and destination IDs; it does not choose execution policy.

Only one task may plan or execute for this robot at a time. A second goal is
rejected while the first owns the motion lease; it never shares mutable
result, event, task, or planning-scene state with the active goal.

The implemented policy rejects concurrent goals. If execution has begun and
then fails, is canceled, or raises an exception, the server latches
`STATE_FAULT` and rejects subsequent goals. The operator must reconcile the
physical robot, gripper, collision object, and attached-object state and then
restart the MTC node. A generic hardware fault reset does not clear this
planning-scene recovery latch.

## `PickPlace` action contract

The interface is `kinova_interfaces/action/PickPlace.action`.

```text
# Goal
string object_id
string destination_id
bool plan_only
---
# Result
bool success
int32 error_code
string message
string failed_stage
uint32 solutions_found
---
# Feedback
string current_stage
uint32 solutions_found
```

### Goal semantics

- `object_id` is an exact key under `objects` in
  `manipulation_objects.json`.
- `destination_id` is an exact key under `destinations`.
- `plan_only=true` permits configuration loading, planning-scene setup, task
  initialization, planning, solution ranking, and introspection publication.
  It must never call task execution or send a controller goal.
- `plan_only=false` executes the selected complete solution only after every
  stage has planned successfully.
- Empty, unknown, or malformed identifiers are rejected before the planning
  scene or robot moves.

### Result semantics

- `success` is true only when the requested operation completed:
  - in plan-only mode, at least one complete solution was found and published;
  - in execution mode, the selected solution returned MoveIt `SUCCESS`.
- `error_code` carries `moveit_msgs/msg/MoveItErrorCodes.val` whenever MoveIt
  produced the failure. MoveIt `SUCCESS` is `1`, not `0`. A failure before
  MoveIt supplies a code uses the middleware's documented generic failure
  value; it must not be reported as success.
- `message` is a concise operator-facing explanation and must include
  actionable context rather than only "planning failed".
- `failed_stage` is the stable stage name from the sequence below. It is empty
  on success.
- `solutions_found` is the number of complete task solutions, not the number
  of IK candidates or partial stage solutions.

### Feedback semantics

The action currently reports coarse, stable task phases:

```text
building task
planning complete task
solution ready
executing best solution
```

- `solutions_found` is zero while building/planning and is updated when
  complete task solutions become available.
- Detailed MTC stage failures are returned in `failed_stage`; they are not
  inferred from the coarse feedback phase.
- Cancellation is checked before and after planning. The required binding
  patch releases Python's GIL during `Task.plan()`, so the action cancel
  callback can call `preempt()` while planning is active.
- Execution sends the selected serialized solution directly to
  `/execute_task_solution` with a Python action client. Cancellation,
  hardware faults, and the execution deadline cancel that goal without
  blocking the executor or telemetry heartbeat.
- Commissioning must still prove that cancellation during controller
  execution stops within a bounded interval; an accepted cancel request alone
  is not proof that the physical robot has stopped.

The internal MTC stage names are listed in the exact task sequence below.
They should remain stable because `failed_stage` exposes them to callers.

## Kinova MTC model contract

The task must use these configured names, not the Panda names from the
tutorial:

| Purpose | Kinova value |
|---|---|
| Arm planning group | `arm` |
| Gripper/end-effector group | `gripper` |
| IK/TCP frame | `tool_frame` |
| Attachment link | `tool_frame` |
| Base/world frame | `base_link` |
| Controlled gripper joint | `right_finger_bottom_joint` |

These names are defaults from `manipulation_objects.json`, not proof that the
deployed SRDF and URDF contain them. Before task initialization, the executor
must confirm that both joint model groups, the IK frame, attachment link,
gripper joint, and every configured touch link exist in the loaded robot
model. A mismatch is a configuration failure, never a reason to substitute a
Panda name.

The Kinova gripper positions are numeric configuration values rather than
assumed Panda SRDF states:

- configured minimum: `0.0`;
- configured maximum: `0.85`;
- initial open seed: `0.0`;
- initial closed seed: `0.8`.

They are commissioning seeds. Confirm their direction, units, physical
clearance, and controller limits on the actual Gen3 Lite before execution.
Never send the legacy unvalidated `1.0` command.

## Exact manual MTC task sequence

Create one fresh task per accepted goal. Load the current robot model and use:

```text
arm stages group     = arm
gripper stages group = gripper
ik_frame             = tool_frame
```

The implementation intentionally does not depend on a named SRDF end
effector or Panda-style named hand states. The Kinova Humble SRDF's
end-effector declaration is unsuitable for copying the tutorial's
`SimpleGrasp` container directly, so the task uses manual stages, an explicit
gripper-joint goal, and explicit touch links.

Use three solver roles:

- a `PipelinePlanner` using the configured `ompl` pipeline and
  `RRTConnectkConfigDefault` for collision-aware free-space connections;
- a joint interpolation solver for the single gripper joint; and
- a `CartesianPath` solver for approach, lift, lower, and retreat.

Velocity and acceleration scaling, Cartesian step size, jump threshold,
minimum accepted fraction, stage timeout, planning attempts, IK limits, and
maximum task solutions come from `manipulation_objects.json`.

The collision object is introduced as an explicit planning-scene stage, so it
is part of the same coherent task solution. The exact implemented stage order
is:

1. **`CurrentState("current state")`**
   - Capture the current robot state and planning scene.
   - Task planning rejects an invalid or colliding start state.

2. **`ModifyPlanningScene("add static collision scene")`**
   - Add or replace every validated object from `obstacles.json`.
   - This makes table/wall geometry part of the task even if the asynchronous
     environment mapper has only just started. The external mapper uses the
     same file, so IDs and geometry remain identical.

3. **`ModifyPlanningScene("add grasp object")`**
   - Add or replace the requested object using its configured ID, shape, and
     `collision_pose`.

4. **`MoveTo("open gripper")`**
   - Group: `gripper`.
   - Goal: `{right_finger_bottom_joint: gripper_open_position}`.
   - Solver: joint interpolation.

5. **`MoveTo("move arm to configured home")`**
   - Group: `arm`.
   - Goal: exact `home_joint_positions` from the manipulation file.
   - Include current-to-home motion in the same collision-checked task.
   - Retain this stage as the monitored pre-grasp state.

6. **`Connect("move to pre-grasp")`**
   - Group: `arm`.
   - Solver: configured planning pipeline with path-length cost.
   - Connect the fixed home state to the exact configured pre-grasp state.
   - Use the bounded `stage_timeout`.

7. **`SerialContainer("pick object")`**

   1. **`MoveRelative("exact vertical pregrasp to grasp")`**
      - Group: `arm`; IK frame/link: `tool_frame`.
      - Solver: Cartesian.
      - Direction: exactly table-normal `-Z` in `base_link`.
      - Distance: exactly the configured `pregrasp_pose` to `grasp_pose`
        vertical separation.
      - Require the configured Cartesian completion fraction. This is the
        stage that prevents a sampled final approach.

   2. **`GeneratePose("generate calibrated grasp pose")`**
      - Emit the configured fixed `grasp_pose` as a `tool_frame` target.
      - Monitor the retained configured-home stage.
      - The implementation deliberately does not sample unconstrained 3-D
        grasp orientations.

   3. **`ComputeIK("compute grasp IK")`**
      - Wrap the calibrated pose generator.
      - Group: `arm`; IK frame: `tool_frame`.
      - Use `max_ik_solutions` and `min_ik_solution_distance`.
      - Reject solutions outside joint limits or in collision.

   4. **`ModifyPlanningScene("allow object collision with gripper")`**
      - Allow collision only between `object_id` and configured
        `touch_links`.
      - Keep collision checking active for the arm and unrelated world
        geometry.

   5. **`MoveTo("close gripper")`**
      - Group: `gripper`.
      - Goal: `{right_finger_bottom_joint: gripper_closed_position}`.
      - Solver: joint interpolation.

   6. **`ModifyPlanningScene("attach object")`**
      - Attach `object_id` to `tool_frame`.

   7. **`ModifyPlanningScene("allow object collision with source support")`**
      - Conditional on object `support_surface`.
      - Allow only the configured object/support pair while contact is
        expected during departure.

   8. **`MoveRelative("lift object")`**
      - Group: `arm`; IK frame: `tool_frame`.
      - Solver: Cartesian.
      - Direction, frame, and distances: object `lift`.
      - Plan with the object attached and require the configured completion
        fraction.

   9. **`ModifyPlanningScene("restore source support collision checking")`**
      - Conditional on object `support_surface`.
      - Restore normal checking immediately after lift.

8. **`Connect("transport object")`**
   - Group: `arm`.
   - Solver: configured planning pipeline with path-length cost.
   - Connect the lifted state to the pre-place state while the object remains
     attached.
   - Use the bounded `stage_timeout`.

9. **`SerialContainer("place object")`**

   1. **`MoveRelative("lower object")`**
      - Group: `arm`; IK frame: `tool_frame`.
      - Solver: Cartesian.
      - Direction, frame, and distances: destination `lower`.
      - It precedes the pose generator so MTC can propagate backward from the
        final place state to a pre-place state; execution remains pre-place to
        final-place.
      - The final pose must leave collision-checked clearance above the tray
        or support. Destination support collision is deliberately not disabled;
        the released item may settle the final small distance under gravity.

   2. **`GeneratePose("generate calibrated place pose")`**
      - Emit destination `tool_pose` directly as the final `tool_frame`
        target.
      - Monitor the completed `pick object` container, whose scene has the
        object attached.

   3. **`ComputeIK("compute place IK")`**
      - Wrap the calibrated pose generator.
      - Group: `arm`; IK frame: `tool_frame`.
      - Apply the configured IK-solution limits.

   4. **`MoveTo("open gripper at destination")`**
      - Group: `gripper`.
      - Goal: `{right_finger_bottom_joint: gripper_open_position}`.

   5. **`ModifyPlanningScene("restore gripper collision checking")`**
      - Restore normal collision checking between the object and every
        configured touch link.

   6. **`ModifyPlanningScene("detach object")`**
      - Detach `object_id` from `tool_frame`.
      - Keep the object in the world at the solved place pose.

   7. **`MoveRelative("retreat after place")`**
      - Group: `arm`; IK frame: `tool_frame`.
      - Solver: Cartesian.
      - Direction, frame, and distances: destination `retreat`.
      - Require the configured completion fraction.

The implemented task deliberately ends after retreat. The next task again
moves to configured home before beginning its grasp sequence.

Call `Task.plan(max_solutions)`, which performs reset and initialization in
the Humble Python binding, then use its cost-ordered complete solutions and
publish the first solution for MTC introspection. If `plan_only` is true,
return at this point. Otherwise serialize exactly that complete solution,
send it to `/execute_task_solution`, and check the returned MoveIt error code.

### Pose-transform rule

All configured arrays use ROS ordering:

- position: `[x, y, z]` in metres;
- quaternion: `[x, y, z, w]`.

`collision_pose` is the object-centre pose. `pregrasp_pose`, `grasp_pose`,
and destination `tool_pose` are `tool_frame` poses. The implemented fixed
`GeneratePose` stages pass those tool targets to `ComputeIK` with
`ik_frame=tool_frame`; they are not object-placement poses from
`GeneratePlacePose`.

For each object, `pregrasp_pose` and `grasp_pose` must:

- use `base_link`;
- have identical normalized orientations;
- place `tool_frame` +Z directly along base-frame -Z, which points the
  Gen3 Lite finger tips toward the table;
- have identical X and Y; and
- differ vertically by 0.02 to 0.25 m.

The loader derives a Cartesian approach whose minimum and maximum distances
are both that exact vertical separation. Humble MTC interprets equal
MoveRelative limits as a required full-distance move, so a partial or lateral
approach cannot satisfy the stage.

If a future implementation changes to an object-pose generator, it must
compute the object/tool rigid transform with TF/Eigen and normalized
quaternions. It must not add/subtract XYZ values or silently reinterpret the
version 1 fields.

## `manipulation_objects.json`

Runtime configuration is installed from:

```text
kinova_interface/data/configs/env/manipulation_objects.json
```

The manipulation file is strict and versioned. The same parser also validates
the legacy-format `obstacles.json` before its table/wall primitives enter an
MTC task. `manipulation_config.py` rejects missing
fields, unknown schema versions, empty/invalid identifiers, zero or
non-normalized quaternions, zero motion vectors, invalid shape dimensions,
unsafe distance ranges, non-downward tool orientations, misaligned
pre-grasp/grasp poses, gripper commands outside configured limits, invalid
planner/scaling values, and unknown object/destination IDs.

The version 1 structure is:

```json
{
  "schema_version": 1,
  "robot": {
    "base_frame": "base_link",
    "arm_group": "arm",
    "home_joint_positions": {
      "joint_1": 0.0,
      "joint_2": 0.0,
      "joint_3": 1.5708,
      "joint_4": 1.5708,
      "joint_5": 1.5708,
      "joint_6": 0.0
    },
    "gripper_group": "gripper",
    "ik_frame": "tool_frame",
    "attach_link": "tool_frame",
    "gripper_joint": "right_finger_bottom_joint",
    "touch_links": ["..."],
    "gripper_min_position": 0.0,
    "gripper_max_position": 0.85,
    "gripper_open_position": 0.0,
    "gripper_closed_position": 0.8,
    "planning_pipeline": "ompl",
    "planner_id": "RRTConnectkConfigDefault",
    "num_planning_attempts": 5,
    "max_solutions": 10,
    "max_ik_solutions": 8,
    "min_ik_solution_distance": 0.5,
    "stage_timeout": 10.0,
    "cartesian_step_size": 0.005,
    "cartesian_jump_threshold": 2.0,
    "cartesian_min_fraction": 1.0,
    "max_velocity_scaling_factor": 0.1,
    "max_acceleration_scaling_factor": 0.1
  },
  "objects": {
    "red_cube": {
      "shape": {
        "type": "box",
        "dimensions": [0.025, 0.025, 0.025]
      },
      "collision_pose": {
        "frame_id": "base_link",
        "position": [-0.3255, -0.1235, 0.01],
        "orientation": [0.0, 0.0, 0.0, 1.0]
      },
      "pregrasp_pose": {
        "frame_id": "base_link",
        "position": [-0.3255, -0.1235, 0.09],
        "orientation": [1.0, 0.0, 0.0, 0.0]
      },
      "grasp_pose": {
        "frame_id": "base_link",
        "position": [-0.3255, -0.1235, 0.01],
        "orientation": [1.0, 0.0, 0.0, 0.0]
      },
      "lift": {
        "frame_id": "base_link",
        "direction": [0.0, 0.0, 1.0],
        "min_distance": 0.06,
        "max_distance": 0.1
      },
      "support_surface": "table"
    }
  },
  "destinations": {
    "delivery_tray": {
      "tool_pose": {
        "frame_id": "base_link",
        "position": [-0.235, -0.425, 0.05],
        "orientation": [1.0, 0.0, 0.0, 0.0]
      },
      "lower": {
        "frame_id": "base_link",
        "direction": [0.0, 0.0, -1.0],
        "min_distance": 0.06,
        "max_distance": 0.1
      },
      "retreat": {
        "frame_id": "base_link",
        "direction": [0.0, 0.0, 1.0],
        "min_distance": 0.08,
        "max_distance": 0.12
      }
    }
  }
}
```

Supported primitive dimensions follow `shape_msgs/SolidPrimitive`:

| Type | Dimension order |
|---|---|
| `box` | `[x, y, z]` |
| `sphere` | `[radius]` |
| `cylinder` | `[height, radius]` |
| `cone` | `[height, radius]` |

### Calibration procedure

Do not treat the committed numeric values as commissioned measurements.

1. Confirm `base_link`, `tool_frame`, gripper links, and finger joint names
   from the live URDF/SRDF.
2. Calibrate the actual TCP. Record the rigid transform from the final wrist
   link to the physical grasp centre.
3. Measure object dimensions with callipers. `collision_pose.position` is the
   primitive centre, not the table-contact point or top surface.
4. Measure the support-surface pose in `base_link` and ensure the object
   primitive does not intersect it.
5. Obtain the object pose in `base_link` from a calibrated perception or
   fixture transform. Static JSON is appropriate only for fixed fixtures.
6. Teach the final `tool_frame` grasp orientation in low-speed/manual mode.
   Normalize the quaternion and inspect its axes in RViz.
7. Teach an exact `pregrasp_pose` directly above `grasp_pose`. Keep X, Y, and
   orientation identical, verify `tool_frame` +Z points toward table -Z, and
   confirm the complete vertical segment is collision-free. Separately verify
   lift, lower, and retreat directions.
8. Determine open and closed gripper commands using the actual controller.
   Closed must grip the object without bottoming out or applying unsafe force.
9. Start with conservative collision padding and 10% velocity/acceleration.
10. Save calibration provenance, date, robot serial number, tool revision,
    camera/fixture revision, and operator with the commissioning record.

Any tool, finger, camera, table, fixture, or payload change invalidates the
relevant calibration gate.

## Failure-risk matrix

| Risk | Observable failure | Required prevention or fix |
|---|---|---|
| MTC Python binding absent or ABI-incompatible on Humble | Import failure, crash, missing stages, or undefined symbols | Build the Humble MTC branch against the exact sourced MoveIt overlay; run a task-construction smoke test. If no supported binding is available, use a C++ MTC action executor behind the unchanged Python action client. |
| `/execute_task_solution` unavailable | Complete plans exist but execution cannot start | Load `move_group/ExecuteTaskSolutionCapability` in `move_group`; verify the action endpoint before accepting execution goals. |
| Panda names copied from the tutorial | Task initialization reports unknown group, state, frame, or link | Use `arm`, `gripper`, and `tool_frame`; validate every configured name against the live Kinova robot model. |
| Gripper group is absent from SRDF or controller mapping | Open/close stage cannot plan or execute | Add/verify the Kinova gripper group, joint limits, end-effector declaration, trajectory/controller mapping, and actual controller state. |
| Unsafe gripper command | Limit violation, object ejection, stall, or hardware fault | Enforce configured min/max and commission open/closed values; inspect controller result rather than assuming success. |
| Object missing or stale in the planning scene | IK passes through it or attach fails | Keep `add grasp object` in the task before any motion, verify its scene diff in the planned solution, and reject mismatched ID/pose data. |
| Duplicate collision-object authority | Object jumps, disappears, or is overwritten | Give the MTC executor one authoritative lifecycle for graspable object IDs; coordinate with the environment mapper. |
| Object starts intersecting its support surface | Start state or grasp IK is in collision | Store the primitive centre correctly, calibrate the table pose, and use only justified contact allowance/padding. |
| Hand/object collision blocks grasp | Grasp IK or close stage fails | Allow collision only for the configured gripper touch links immediately before closing; restore it before detach. |
| Collision allowance is too broad or not restored | Arm/support collisions are ignored after the contact phase | Never allow object collision with the arm or unrelated world geometry. Allow only the exact configured source support during departure, restore it immediately after lift, and keep destination support collision checking enabled. |
| Incorrect TCP or grasp transform | Consistent lateral/vertical miss despite valid plans | Recalibrate `tool_frame`; compute rigid transforms with TF/Eigen and use the configured fixed grasp orientation. |
| Unconstrained sampled final approach | Different paths near a small object | Use Cartesian `MoveRelative` for approach and require the configured full-path fraction; use sampling only for `Connect`. |
| Cartesian discontinuity or partial path | Low fraction, joint jump, singularity, or collision | Tune step size/jump threshold and distances, select another IK solution, and reject every path below `cartesian_min_fraction`. |
| No IK solution | `grasp pose IK` or `place pose IK` has zero solutions | Check reachability, frame/quaternion, TCP transform, object centre, joint limits, and collision geometry before increasing samples. |
| Transfer path is still variable | Different collision-free routes between pick and place | This is expected for a sampling connector. Fix planner/pipeline parameters and cost selection; require deterministic Cartesian motion only in the near-object segments. |
| MoveIt or planning scene not ready at startup | One-time setup silently skips or first goal fails | Use bounded readiness checks with retry/backoff; reject goals until all required services/actions and the robot model are ready. |
| Concurrent goals corrupt state | Cross-completed results, wrong attached object, or scene races | Serialize goals with one execution guard and use per-goal task/result/cancellation objects. |
| Unbounded wait | Action never returns after callback or controller failure | Set explicit planning, execution, scene, and cancellation deadlines; convert timeout to a failed result and safe cleanup. |
| Unpatched Humble Python binding holds the GIL | Heartbeats and planning cancellation pause during `Task.plan()` | Apply the pinned repository patch before building; startup verifies its exported marker. Execution uses a cancellable Python client for `/execute_task_solution`, not the blocking `Task.execute()` binding. |
| Cancellation while object is attached | Scene and real robot disagree | Stop execution, query actual state, preserve/repair attachment state deliberately, open only when physically safe, and require operator recovery if state is uncertain. |
| Hardware fault during execution | Protective stop or controller abort | Abort the action, preserve the exact failed stage/error, publish telemetry fault, and require the existing reset workflow plus scene/state reconciliation. |
| Plan-only accidentally executes | Robot moves during validation | Keep execution behind one explicit `if not plan_only` guard and test that no controller/action goal is emitted. |
| Tutorial/API version drift | Compile/import errors or changed stage behavior | Pin the MTC branch to `humble`, record MoveIt/MTC commits or Debian versions, and do not code against Rolling `main` without port verification. |

## Dependency and build checklist

Run these checks on Ubuntu 22.04, not this Windows checkout.

- [ ] Source `/opt/ros/humble/setup.bash`.
- [ ] Source the Kinova Kortex/MoveIt workspace that supplies
      `kortex_bringup` and `kinova_gen3_lite_moveit_config`.
- [ ] Build the pinned MTC `humble` commit in the same overlay as the
      deployed MoveIt version and apply the required GIL/timeout patch.
- [ ] Build the `ros2` branch of `py_binding_tools` at the pinned commit
      below; its default/ROS 1 branch is incompatible.
- [ ] Resolve dependencies with `rosdep install --from-paths src --ignore-src
      --rosdistro humble`.
- [ ] Confirm `moveit_task_constructor_core`,
      `moveit_task_constructor_capabilities`, `py_binding_tools`,
      `moveit_task_constructor_msgs`, and the visualization package are
      discoverable.
- [ ] For a C++ executor, declare and link the exact MTC, `rclcpp`,
      MoveIt planning interface, geometry/shape message, and TF dependencies.
- [ ] For the implemented Python executor, prove both `import rclcpp` and
      `from moveit.task_constructor import core, stages`; then construct
      `Task`, all required solvers, and every required stage before claiming
      Python support.
- [ ] Ensure `PickPlace.action` is listed by
      `kinova_interfaces/CMakeLists.txt` and the interface package declares
      action dependencies.
- [ ] Ensure `manipulation_objects.json` is installed with the package data.
- [ ] Build at least `kinova_interfaces` before `kinova_interface` and any
      MTC executor package.
- [ ] Source the new `install/setup.bash` in every runtime terminal.
- [ ] Run the ROS-independent configuration tests.
- [ ] Run a clean `colcon build` with no reliance on stale `build/`,
      `install/`, or `log/` artifacts.

Typical source-build setup for MTC follows the official Humble branch:

```bash
cd ~/workspace/ros2_kortex_ws/src
git clone -b humble https://github.com/moveit/moveit_task_constructor.git
git -C moveit_task_constructor checkout 756634951326ae17ae099882f7110c6f1d0a98c0
git clone -b ros2 https://github.com/moveit/py_binding_tools.git
git -C py_binding_tools checkout 8918f0ece3d7977a4963c54472531906e01e40c2
git -C moveit_task_constructor apply \
  /absolute/path/to/ROS2-middleware/patches/moveit_task_constructor_humble_release_gil.patch
cd ..
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --packages-up-to kinova_interfaces kinova_interface
source install/setup.bash
```

Adjust package selection if the executor lives in a separate C++ package.
Never mix a Rolling MTC build with Humble MoveIt libraries.

Run this exact non-motion ABI check before launching the middleware:

```bash
python3 - <<'PY'
import rclcpp
from moveit.task_constructor import core, stages

rclcpp.init()
options = rclcpp.NodeOptions(
    automatically_declare_parameters_from_overrides=True,
    enable_rosout=False,
)
node = rclcpp.Node("mtc_abi_smoke", options)
planner = core.PipelinePlanner(node, "ompl")
task = core.Task()
task.add(stages.CurrentState("current"))
assert hasattr(task, "loadRobotModel")
assert hasattr(task, "plan")
assert hasattr(task, "publish")
assert hasattr(task, "execute")
assert hasattr(task, "timeout")
assert core.blocking_calls_release_gil is True
print("MTC Humble Python ABI smoke test passed")
del task, planner, node
rclcpp.shutdown()
PY
```

If this fails, do not launch the Python MTC node. Confirm the inspected commit,
clean the overlay, and rebuild. If the deployment must use an older binary
whose bindings lack this API, retain the `PickPlace` action contract and
implement its MTC worker in C++.

## Runtime readiness checklist

- [ ] `/robot_description` and `/robot_description_semantic` are loaded.
- [ ] The live SRDF contains groups `arm` and `gripper`.
- [ ] TF resolves `base_link` to `tool_frame` continuously.
- [ ] All configured touch links and `right_finger_bottom_joint` exist.
- [ ] `move_group` is running with the `ompl` pipeline and configured planner.
- [ ] `move_group/ExecuteTaskSolutionCapability` is loaded.
- [ ] `/execute_task_solution`, `/move_action`, and the trajectory execution
      endpoints are available.
- [ ] Arm and `gen3_lite_2f_gripper_controller` controllers are active.
- [ ] `/apply_planning_scene` responds and scene updates are observed.
- [ ] The planned solution's `add static collision scene` diff contains the
      validated table and wall before arm motion.
- [ ] The MTC executor reports ready and its PickPlace action is listed.
- [ ] Telemetry reports no stale node, controller warning, or hardware fault.
- [ ] The selected object and destination resolve through the strict config.

## Plan-only checklist

Use `plan_only=true` before every new object, destination, calibration, or
software release.

- [ ] The action returns a complete solution and `solutions_found > 0`.
- [ ] Feedback stage names progress in the documented order.
- [ ] The RViz Motion Planning Tasks display shows the selected solution.
- [ ] Approach, lift, lower, and retreat are straight Cartesian segments in
      their configured frames.
- [ ] The grasp keeps `tool_frame` at the calibrated orientation.
- [ ] No waypoint is in self, table, wall, object, or fixture collision except
      explicitly allowed gripper/object contact.
- [ ] The object is attached for lift and transfer in the task's planning
      scenes and detached for retreat.
- [ ] No controller goal is sent and no robot joint changes.
- [ ] Unknown object/destination IDs and unsafe config return failure before
      planning.
- [ ] A zero quaternion, zero direction, out-of-range gripper value, and
      partial Cartesian path are rejected.
- [ ] Cancellation returns within its deadline and leaves no stale task lock.

Example invocation shape:

```bash
ros2 action send_goal /mtc_task_node/pick_place kinova_interfaces/action/PickPlace \
  "{object_id: red_cube, destination_id: delivery_tray, plan_only: true}" \
  --feedback
```

The default is `/mtc_task_node/pick_place`; use the launch-configured action
name if it is overridden or namespaced.

## Fake-hardware checklist

- [ ] Launch the complete stack with `use_fake_hardware:=true`.
- [ ] Confirm the correct Gen3 Lite gripper controller is active.
- [ ] Pass every plan-only check first.
- [ ] Execute one solution with `plan_only=false`.
- [ ] Verify joint trajectories execute in stage order.
- [ ] Verify scene attachment occurs after close and persists through transfer.
- [ ] Verify forbid-collision and detach occur before retreat.
- [ ] Repeat from representative collision-free arm start states.
- [ ] Exercise invalid IDs, no-IK, blocked approach, planning timeout,
      execution failure, and cancellation.
- [ ] Confirm each failure reports `failed_stage`, an error code, and a useful
      message without hanging the executor.
- [ ] Confirm a second simultaneous goal cannot interfere with the first.
- [ ] Confirm subsequent valid goals work after every simulated failure.

## Physical-hardware commissioning checklist

- [ ] Obtain lab authorization, use a trained operator/spotter, and keep the
      emergency stop immediately accessible.
- [ ] Inspect the arm, gripper, item, fixtures, table, cables, and workspace.
- [ ] Connect with the explicitly supplied robot IP.
- [ ] Confirm the fault controller is active and the reset path works before
      manipulation.
- [ ] Confirm actual joint state agrees with RViz and the planning scene.
- [ ] Complete and record TCP, object, destination, surface, and gripper
      calibration.
- [ ] Set payload/tool data appropriate for the physical gripper and item.
- [ ] Keep velocity and acceleration at the initial 10% limits.
- [ ] Plan only and visually approve the exact selected solution.
- [ ] First execute without an object, stopping before gripper contact if
      practical.
- [ ] Execute approach-only/retreat-only commissioning motions at low speed.
- [ ] Perform the first grasp with maximum clearance and a single small item.
- [ ] Confirm the item is secure before permitting transfer.
- [ ] Confirm release height and tray clearance before opening.
- [ ] Reconcile planning-scene attachment after every abort or manual
      intervention.
- [ ] Increase speed only after the acceptance gates pass; calibration, not
      speed, is the priority.

## Acceptance gates

The integration is not complete merely because one demo succeeds.

### Gate A: interface and configuration

- Clean Humble build succeeds from an empty build/install/log state.
- The generated `PickPlace` action is importable and discoverable.
- The committed manipulation config passes its unit tests.
- Every configured group, frame, joint, link, planner, and controller is
  verified against the live system.
- Invalid IDs and all unsafe configuration fixtures fail before motion.

### Gate B: planning

- At least one complete plan is found from every agreed representative start
  state.
- Twenty consecutive plan-only requests for each commissioned
  object/destination pair complete without an unexplained stage failure.
- Every near-object segment is Cartesian and meets
  `cartesian_min_fraction`.
- RViz inspection confirms no forbidden collision and correct attach/detach
  state.
- `plan_only=true` produces zero controller goals in automated evidence.

### Gate C: fake hardware

- Ten consecutive full executions per object/destination pair succeed.
- Cancellation, timeout, blocked-path, no-IK, controller-failure, and
  concurrent-goal tests terminate safely and leave the next goal operable.
- Feedback, `failed_stage`, error code, solution count, and telemetry agree
  with the observed outcome.
- No wait is unbounded and no stale collision/attachment state remains.

### Gate D: physical hardware

- A lab-approved dry run without payload completes at low speed.
- Ten consecutive picks and places per commissioned small item succeed from
  the agreed starting envelope without contact outside the intended finger
  surfaces.
- The final approach has no visible lateral sweep and stays within the
  calibrated positional tolerance selected by the lab.
- No protective stop, joint-limit event, dropped item, fixture contact, or
  unreported controller failure occurs.
- Every failure deliberately injected by the commissioning procedure stops
  safely and reports the exact stage.

### Gate E: operational release

- Calibration records, exact ROS/MoveIt/MTC/Kortex versions, logs, and test
  evidence are archived.
- The README/runbook points operators to this guide.
- Recovery from an attached-object abort is documented and rehearsed.
- Any change to robot description, tool, gripper, planner, scene geometry,
  object geometry, destination, or calibration reruns the affected gates.

Until all applicable gates pass, the action remains restricted to plan-only,
fake hardware, or supervised low-speed commissioning as appropriate.
