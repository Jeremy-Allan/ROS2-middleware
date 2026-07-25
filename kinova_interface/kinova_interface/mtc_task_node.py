"""MoveIt Task Constructor pick-and-place action server for the Gen3 Lite."""

from __future__ import annotations

import os
import threading
import time
from typing import Iterable

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from example_interfaces.msg import Bool
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from moveit_msgs.msg import CollisionObject, MoveItErrorCodes
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from moveit_task_constructor_msgs.action import ExecuteTaskSolution

from kinova_interfaces.action import PickPlace
from kinova_interfaces.msg import ExtendedStatus
from kinova_interface.manipulation_config import (
    ConfigurationError,
    DestinationSpec,
    ManipulationConfig,
    MotionSpec,
    ObjectSpec,
    PoseSpec,
    SceneObjectSpec,
    load_manipulation_config,
    load_scene_objects,
)
from kinova_interface.motion_lease import MotionLease, MotionLeaseError

try:
    import rclcpp
    from moveit.task_constructor import core, stages

    _MTC_IMPORT_ERROR = None
except ImportError as exc:  # pragma: no cover - exercised by the ROS ABI smoke test
    rclcpp = None
    core = None
    stages = None
    _MTC_IMPORT_ERROR = exc


class MtcTaskNode(Node):
    """Build, plan, inspect, and execute one atomic MTC manipulation task."""

    def __init__(self):
        if _MTC_IMPORT_ERROR is not None:
            raise RuntimeError(
                "MoveIt Task Constructor Python bindings are unavailable. "
                "Install moveit_task_constructor_core and py_binding_tools from the "
                f"same ROS/MoveIt build: {_MTC_IMPORT_ERROR}"
            )
        if not getattr(core, "blocking_calls_release_gil", False):
            raise RuntimeError(
                "The MTC Python binding does not release the GIL during Task.plan(). "
                "Apply patches/moveit_task_constructor_humble_release_gil.patch "
                "to the pinned Humble source and rebuild the overlay."
            )

        super().__init__(
            "mtc_task_node",
            automatically_declare_parameters_from_overrides=True,
        )

        default_config = os.path.join(
            get_package_share_directory("kinova_interface"),
            "data",
            "configs",
            "env",
            "manipulation_objects.json",
        )
        config_path = self._parameter("manipulation_config", default_config)
        action_name = self._parameter(
            "pick_place_action", "/mtc_task_node/pick_place"
        )
        self._allow_execution = self._parameter("allow_mtc_execution", False)
        self._execution_timeout = float(
            self._parameter("mtc_execution_timeout_sec", 300.0)
        )
        self._planning_timeout = float(
            self._parameter("mtc_planning_timeout_sec", 120.0)
        )
        motion_lock_path = self._parameter(
            "motion_lock_path",
            "/tmp/kinova_gen3_lite_motion.lock",
        )
        if not isinstance(self._allow_execution, bool):
            raise ConfigurationError("allow_mtc_execution must be true or false")
        if self._execution_timeout <= 0.0 or self._planning_timeout <= 0.0:
            raise ConfigurationError(
                "MTC planning/execution timeouts must be positive"
            )

        self._config = load_manipulation_config(config_path)
        self._scene_objects = load_scene_objects(
            os.path.join(os.path.dirname(config_path), "obstacles.json")
        )
        scene_ids = {item.object_id for item in self._scene_objects}
        duplicate_ids = set(self._config.objects).intersection(
            scene_ids
        )
        if duplicate_ids:
            raise ConfigurationError(
                "Pickable object IDs conflict with static scene IDs: "
                + ", ".join(sorted(duplicate_ids))
            )
        missing_surfaces = {
            item.support_surface
            for item in self._config.objects.values()
            if item.support_surface and item.support_surface not in scene_ids
        }
        if missing_surfaces:
            raise ConfigurationError(
                "Object support surfaces are absent from obstacles.json: "
                + ", ".join(sorted(missing_surfaces))
            )

        # Humble's MTC bindings require an rclcpp node. Node-local arguments give it
        # a unique graph name while retaining launch-provided global parameter files.
        options = rclcpp.NodeOptions(
            automatically_declare_parameters_from_overrides=True,
            enable_rosout=False,
        )
        options.arguments = [
            "--ros-args",
            "-r",
            "__node:=mtc_task_context",
        ]
        self._mtc_node = rclcpp.Node("mtc_task_context", options)
        self._motion_lease = MotionLease(motion_lock_path, self.get_name())

        self._state_lock = threading.Lock()
        self._busy = False
        self._active_task = None
        self._active_execution_goal = None
        self._recovery_required = False
        self._hardware_faulted = False
        self._hardware_interface_ready = False
        self._hardware_status_at = 0.0
        self._preflight_ready = False
        self._last_command_valid = False
        self._state = ExtendedStatus.STATE_BUSY
        self._status_text = "MTC startup preflight pending"

        self._callback_group = ReentrantCallbackGroup()
        self._status_publisher = self.create_publisher(
            ExtendedStatus, "/status/node_report", 10
        )
        self._status_timer = self.create_timer(
            0.5, self._publish_status, callback_group=self._callback_group
        )
        self._execute_solution_client = ActionClient(
            self,
            ExecuteTaskSolution,
            "/execute_task_solution",
            callback_group=self._callback_group,
        )
        self._fault_subscription = self.create_subscription(
            Bool,
            "/fault_controller/is_faulted",
            self._fault_callback,
            10,
            callback_group=self._callback_group,
        )
        self._hardware_status_subscription = self.create_subscription(
            ExtendedStatus,
            "/status/node_report",
            self._hardware_status_callback,
            10,
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            PickPlace,
            action_name,
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=self._callback_group,
        )
        self._preflight_timer = self.create_timer(
            1.0,
            self._run_preflight,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            f"MTC pick/place action starting at {action_name}; "
            f"loaded {len(self._config.objects)} objects and "
            f"{len(self._config.destinations)} destinations plus "
            f"{len(self._scene_objects)} static scene objects from {config_path}; "
            f"commissioned={self._config.commissioned}, "
            f"execution_enabled={self._allow_execution}"
        )

    def _parameter(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _set_status(self, state, text, valid):
        self._state = state
        self._status_text = text
        self._last_command_valid = valid
        self._publish_status()

    def _publish_status(self):
        message = ExtendedStatus()
        message.node_name = self.get_name()
        message.state = self._state
        message.status_message = self._status_text
        message.last_command_valid = self._last_command_valid
        self._status_publisher.publish(message)

    def _run_preflight(self):
        if self._preflight_ready:
            return
        if not self._execute_solution_client.server_is_ready():
            self._set_status(
                ExtendedStatus.STATE_BUSY,
                "Waiting for /execute_task_solution capability",
                False,
            )
            return

        try:
            robot = self._config.robot
            planner = core.PipelinePlanner(
                self._mtc_node,
                robot.planning_pipeline,
            )
            planner.planner = robot.planner_id

            # Initializing a connector between two generators forces MTC to load
            # the robot model, groups, and selected planning-pipeline plugin
            # without producing or executing any trajectory.
            preflight = core.Task("", False)
            preflight.timeout = self._planning_timeout
            preflight.loadRobotModel(self._mtc_node)
            preflight.add(stages.CurrentState("preflight current"))
            connector = stages.Connect(
                "preflight planning pipeline",
                [(robot.arm_group, planner)],
            )
            connector.timeout = robot.stage_timeout
            preflight.add(connector)
            preflight.add(stages.CurrentState("preflight target"))
            preflight.init()
        except Exception as exc:  # noqa: BLE001 - report binding/plugin diagnostics
            self._set_status(
                ExtendedStatus.STATE_FAULT,
                f"MTC startup preflight failed: {exc}",
                False,
            )
            return

        self._preflight_ready = True
        self._preflight_timer.cancel()
        if self._hardware_faulted:
            self._set_status(
                ExtendedStatus.STATE_FAULT,
                "MTC preflight passed, but robot hardware is faulted",
                False,
            )
        else:
            self._set_status(
                ExtendedStatus.STATE_IDLE,
                "MTC preflight passed; task server idle",
                True,
            )
        self.get_logger().info(
            "MTC preflight passed: robot model, planning pipeline, and "
            "/execute_task_solution are ready"
        )

    def _hardware_status_callback(self, message: ExtendedStatus):
        if message.node_name != "kinova_hardware_client":
            return
        self._hardware_interface_ready = (
            message.state == ExtendedStatus.STATE_IDLE
            and message.last_command_valid
        )
        self._hardware_status_at = time.monotonic()

    def _hardware_motion_ready(self):
        return (
            self._hardware_interface_ready
            and time.monotonic() - self._hardware_status_at <= 2.0
        )

    def _fault_callback(self, message: Bool):
        self._hardware_faulted = bool(message.data)
        if not message.data:
            if self._recovery_required:
                self._set_status(
                    ExtendedStatus.STATE_FAULT,
                    "Hardware fault cleared, but executed task state still "
                    "requires reconciliation and restart",
                    False,
                )
            elif self._preflight_ready and not self._busy:
                self._set_status(
                    ExtendedStatus.STATE_IDLE,
                    "Robot hardware fault cleared; MTC task server idle",
                    True,
                )
            return

        self.get_logger().error(
            "Robot hardware fault received; preempting MTC planning/execution"
        )
        with self._state_lock:
            active_task = self._active_task
            execution_goal = self._active_execution_goal
            if execution_goal is not None:
                self._recovery_required = True
        if active_task is not None:
            try:
                active_task.preempt()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"Could not preempt faulted MTC task: {exc}")
        if execution_goal is not None:
            try:
                execution_goal.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f"Could not cancel faulted solution execution: {exc}"
                )
        self._set_status(
            ExtendedStatus.STATE_FAULT,
            "HARDWARE FAULT: MTC planning/execution preempt requested",
            False,
        )

    def _goal(self, goal_request):
        try:
            if not goal_request.object_id.strip():
                raise ConfigurationError("object_id cannot be empty")
            if not goal_request.destination_id.strip():
                raise ConfigurationError("destination_id cannot be empty")
            self._config.resolve(
                goal_request.object_id.strip(),
                goal_request.destination_id.strip(),
            )
        except ConfigurationError as exc:
            self.get_logger().error(f"Rejecting invalid pick/place goal: {exc}")
            return GoalResponse.REJECT

        if not self._preflight_ready:
            self.get_logger().error(
                "Rejecting pick/place goal because MTC startup preflight has "
                "not passed"
            )
            return GoalResponse.REJECT
        if self._hardware_faulted:
            self.get_logger().error(
                "Rejecting pick/place goal because robot hardware is faulted"
            )
            return GoalResponse.REJECT
        if not goal_request.plan_only and not self._allow_execution:
            self.get_logger().error(
                "Rejecting execution goal: launch with "
                "allow_mtc_execution:=true only after commissioning"
            )
            return GoalResponse.REJECT
        if not goal_request.plan_only and not self._config.commissioned:
            self.get_logger().error(
                "Rejecting execution goal because manipulation_objects.json "
                "has commissioned=false"
            )
            return GoalResponse.REJECT
        if not goal_request.plan_only and not self._hardware_motion_ready():
            self.get_logger().error(
                "Rejecting execution goal because the hardware interface has "
                "not reported fresh, fault-monitored motion readiness"
            )
            return GoalResponse.REJECT
        if (
            not goal_request.plan_only
            and not self._execute_solution_client.wait_for_server(timeout_sec=0.5)
        ):
            self.get_logger().error(
                "Rejecting execution goal because /execute_task_solution is "
                "unavailable. Ensure MoveGroup loaded "
                "move_group/ExecuteTaskSolutionCapability."
            )
            return GoalResponse.REJECT

        with self._state_lock:
            if self._recovery_required:
                self.get_logger().error(
                    "Rejecting pick/place goal because a previous execution "
                    "failed after motion began; reconcile robot/scene state and "
                    "restart the MTC node"
                )
                return GoalResponse.REJECT
            if self._busy:
                self.get_logger().warning(
                    "Rejecting pick/place goal because another task is active"
                )
                return GoalResponse.REJECT
            try:
                acquired = self._motion_lease.acquire(
                    f"pick_place:{goal_request.object_id.strip()}"
                )
            except MotionLeaseError as exc:
                self.get_logger().error(f"Rejecting goal: {exc}")
                return GoalResponse.REJECT
            if not acquired:
                self.get_logger().warning(
                    "Rejecting pick/place goal because another middleware "
                    "process owns the robot motion lease"
                )
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _cancel(self, _goal_handle):
        with self._state_lock:
            active_task = self._active_task
            execution_goal = self._active_execution_goal
        if active_task is not None:
            try:
                active_task.preempt()
            except Exception as exc:  # noqa: BLE001 - keep cancellation fail-safe
                self.get_logger().error(f"Could not preempt active MTC task: {exc}")
        if execution_goal is not None:
            try:
                execution_goal.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001 - keep cancellation fail-safe
                self.get_logger().error(
                    f"Could not cancel active MTC execution: {exc}"
                )
        return CancelResponse.ACCEPT

    def _feedback(self, goal_handle, stage, solutions=0):
        feedback = PickPlace.Feedback()
        feedback.current_stage = stage
        feedback.solutions_found = solutions
        goal_handle.publish_feedback(feedback)
        self._status_text = stage
        self._publish_status()

    def _result(self, success, code, message, stage="", solutions=0):
        result = PickPlace.Result()
        result.success = success
        result.error_code = int(code)
        result.message = message
        result.failed_stage = stage
        result.solutions_found = solutions
        return result

    def _execution_feedback(self, goal_handle, feedback_message, solutions):
        try:
            sub_no = feedback_message.feedback.sub_no
            self._feedback(
                goal_handle,
                f"executing solution subtrajectory {sub_no}",
                solutions,
            )
        except Exception as exc:  # noqa: BLE001 - late feedback must not abort
            self.get_logger().warning(
                f"Could not publish MTC execution feedback: {exc}"
            )

    def _cancel_late_execution_submission(self, future):
        """Cancel a goal whose server response arrived after our deadline."""

        try:
            goal_handle = future.result()
            if goal_handle is not None and goal_handle.accepted:
                goal_handle.cancel_goal_async()
                self.get_logger().error(
                    "Canceled a late-accepted MTC execution goal; operator "
                    "recovery remains required"
                )
        except Exception as exc:  # noqa: BLE001 - best-effort containment
            self.get_logger().error(
                f"Could not cancel late MTC execution submission: {exc}"
            )

    def _execute_solution(self, goal_handle, task, solution, solution_count):
        """Execute through rclpy so fault/cancel callbacks remain responsive."""

        execute_goal = ExecuteTaskSolution.Goal()
        execute_goal.solution = solution.toMsg(task.introspection())
        send_future = self._execute_solution_client.send_goal_async(
            execute_goal,
            feedback_callback=lambda message: self._execution_feedback(
                goal_handle,
                message,
                solution_count,
            ),
        )

        response_deadline = time.monotonic() + 10.0
        while rclpy.ok() and not send_future.done():
            early_stop = None
            early_code = MoveItErrorCodes.PREEMPTED
            early_outcome = "failure"
            if self._hardware_faulted:
                early_stop = "hardware fault during MTC goal submission"
            elif not self._hardware_motion_ready():
                early_stop = "hardware readiness loss during MTC goal submission"
                early_code = MoveItErrorCodes.CONTROL_FAILED
            elif goal_handle.is_cancel_requested:
                early_stop = "client cancellation during MTC goal submission"
                early_outcome = "canceled"
            if early_stop is not None:
                send_future.add_done_callback(
                    self._cancel_late_execution_submission
                )
                return (early_outcome, early_code, early_stop, True)
            if time.monotonic() >= response_deadline:
                # The request may already be at MoveGroup. Do not cancel only
                # the local Future and orphan a possibly accepted robot goal.
                send_future.add_done_callback(
                    self._cancel_late_execution_submission
                )
                return (
                    "failure",
                    MoveItErrorCodes.TIMED_OUT,
                    "Timed out waiting for MoveGroup to accept the MTC "
                    "solution; any late acceptance will be canceled",
                    True,
                )
            time.sleep(0.02)

        if not send_future.done():
            return (
                "failure",
                MoveItErrorCodes.PREEMPTED,
                "ROS shutdown interrupted MTC solution submission",
                True,
            )
        try:
            execution_goal = send_future.result()
        except Exception as exc:  # noqa: BLE001
            return (
                "failure",
                MoveItErrorCodes.CONTROL_FAILED,
                f"Could not submit MTC solution: {exc}",
                True,
            )
        if execution_goal is None or not execution_goal.accepted:
            return (
                "failure",
                MoveItErrorCodes.CONTROL_FAILED,
                "MoveGroup rejected the MTC solution",
                False,
            )

        with self._state_lock:
            self._active_execution_goal = execution_goal
        result_future = execution_goal.get_result_async()
        execution_deadline = time.monotonic() + self._execution_timeout
        cancellation_reason = None
        cancellation_deadline = None

        try:
            while rclpy.ok() and not result_future.done():
                now = time.monotonic()
                requested_reason = None
                if self._hardware_faulted:
                    requested_reason = "hardware fault"
                elif not self._hardware_motion_ready():
                    requested_reason = "hardware readiness loss"
                elif goal_handle.is_cancel_requested:
                    requested_reason = "client cancellation"
                elif now >= execution_deadline:
                    requested_reason = "execution timeout"

                if requested_reason and cancellation_reason is None:
                    cancellation_reason = requested_reason
                    cancellation_deadline = now + 5.0
                    try:
                        execution_goal.cancel_goal_async()
                    except Exception as exc:  # noqa: BLE001
                        self.get_logger().error(
                            f"Failed to request solution cancellation: {exc}"
                        )

                if (
                    cancellation_deadline is not None
                    and now >= cancellation_deadline
                ):
                    if cancellation_reason == "execution timeout":
                        code = MoveItErrorCodes.TIMED_OUT
                    elif cancellation_reason == "hardware readiness loss":
                        code = MoveItErrorCodes.CONTROL_FAILED
                    else:
                        code = MoveItErrorCodes.PREEMPTED
                    return (
                        "failure",
                        code,
                        f"MTC {cancellation_reason} was not confirmed within 5 seconds",
                        True,
                    )
                time.sleep(0.02)

            if not result_future.done():
                return (
                    "failure",
                    MoveItErrorCodes.PREEMPTED,
                    "ROS shutdown interrupted MTC execution",
                    True,
                )
            try:
                wrapped_result = result_future.result()
            except Exception as exc:  # noqa: BLE001
                return (
                    "failure",
                    MoveItErrorCodes.CONTROL_FAILED,
                    f"MTC execution result failed: {exc}",
                    True,
                )

            execution_code = wrapped_result.result.error_code.val
            if (
                wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
                and execution_code == MoveItErrorCodes.SUCCESS
            ):
                return ("success", execution_code, "MTC solution executed", False)

            if cancellation_reason == "client cancellation":
                return (
                    "canceled",
                    MoveItErrorCodes.PREEMPTED,
                    "Pick/place execution was canceled",
                    True,
                )
            if cancellation_reason == "hardware fault":
                return (
                    "failure",
                    MoveItErrorCodes.PREEMPTED,
                    "Hardware fault interrupted pick/place execution",
                    True,
                )
            if cancellation_reason == "hardware readiness loss":
                return (
                    "failure",
                    MoveItErrorCodes.CONTROL_FAILED,
                    "Hardware readiness heartbeat was lost during execution",
                    True,
                )
            if cancellation_reason == "execution timeout":
                return (
                    "failure",
                    MoveItErrorCodes.TIMED_OUT,
                    "Pick/place execution exceeded its deadline",
                    True,
                )
            return (
                "failure",
                execution_code,
                "MoveGroup failed while executing the MTC solution",
                True,
            )
        finally:
            with self._state_lock:
                self._active_execution_goal = None

    def _execute(self, goal_handle):
        request = goal_handle.request
        object_id = request.object_id.strip()
        destination_id = request.destination_id.strip()
        task = None
        solution_count = 0
        execution_started = False

        try:
            object_spec, destination = self._config.resolve(
                object_id, destination_id
            )
            operation = f"{object_id} -> {destination_id}"
            self._set_status(
                ExtendedStatus.STATE_BUSY,
                f"Building MTC task: {operation}",
                True,
            )
            self._feedback(goal_handle, "building task")

            task = self._build_task(object_spec, destination)
            with self._state_lock:
                self._active_task = task

            if goal_handle.is_cancel_requested:
                task.preempt()
                goal_handle.canceled()
                self._last_command_valid = False
                return self._result(
                    False,
                    MoveItErrorCodes.PREEMPTED,
                    "Pick/place canceled before planning",
                    "building task",
                )

            self._feedback(goal_handle, "planning complete task")
            planning_result = task.plan(self._config.robot.max_solutions)
            solution_count = len(task.solutions)
            if goal_handle.is_cancel_requested:
                task.preempt()
                goal_handle.canceled()
                self._last_command_valid = False
                return self._result(
                    False,
                    MoveItErrorCodes.PREEMPTED,
                    "Pick/place canceled during planning",
                    "planning complete task",
                    solution_count,
                )
            if self._hardware_faulted:
                task.preempt()
                goal_handle.abort()
                self._last_command_valid = False
                return self._result(
                    False,
                    MoveItErrorCodes.PREEMPTED,
                    "Hardware fault interrupted MTC planning",
                    "planning complete task",
                    solution_count,
                )
            if not bool(planning_result) or solution_count == 0:
                failed_stage, detail = self._find_failure(task)
                code = getattr(
                    planning_result,
                    "val",
                    MoveItErrorCodes.PLANNING_FAILED,
                )
                message = "No complete MTC solution was found"
                if detail:
                    message += f": {detail}"
                goal_handle.abort()
                self._last_command_valid = False
                return self._result(
                    False,
                    code,
                    message,
                    failed_stage,
                    solution_count,
                )

            self._feedback(goal_handle, "solution ready", solution_count)
            solution = task.solutions[0]
            task.publish(solution)

            if goal_handle.is_cancel_requested:
                task.preempt()
                goal_handle.canceled()
                self._last_command_valid = False
                return self._result(
                    False,
                    MoveItErrorCodes.PREEMPTED,
                    "Pick/place canceled after planning",
                    "solution ready",
                    solution_count,
                )

            if request.plan_only:
                goal_handle.succeed()
                self._last_command_valid = True
                return self._result(
                    True,
                    MoveItErrorCodes.SUCCESS,
                    f"Plan-only succeeded with {solution_count} complete solution(s)",
                    "",
                    solution_count,
                )

            if not self._hardware_motion_ready():
                goal_handle.abort()
                self._last_command_valid = False
                return self._result(
                    False,
                    MoveItErrorCodes.CONTROL_FAILED,
                    "Hardware motion readiness was lost while planning; "
                    "the solution was not executed",
                    "solution ready",
                    solution_count,
                )

            self._feedback(goal_handle, "executing best solution", solution_count)
            execution_started = True
            outcome, execution_code, execution_message, uncertain = (
                self._execute_solution(
                    goal_handle,
                    task,
                    solution,
                    solution_count,
                )
            )
            if uncertain:
                with self._state_lock:
                    self._recovery_required = True
            if outcome == "canceled":
                goal_handle.canceled()
                self._last_command_valid = False
                return self._result(
                    False,
                    execution_code,
                    execution_message,
                    "executing best solution",
                    solution_count,
                )
            if outcome != "success":
                goal_handle.abort()
                self._last_command_valid = False
                return self._result(
                    False,
                    execution_code,
                    execution_message
                    + "; inspect the robot and planning scene before retrying",
                    "executing best solution",
                    solution_count,
                )

            goal_handle.succeed()
            self._last_command_valid = True
            return self._result(
                True,
                execution_code,
                f"Pick/place completed: {operation}",
                "",
                solution_count,
            )
        except ConfigurationError as exc:
            goal_handle.abort()
            self._last_command_valid = False
            return self._result(
                False,
                MoveItErrorCodes.INVALID_MOTION_PLAN,
                str(exc),
                "configuration",
                solution_count,
            )
        except Exception as exc:  # noqa: BLE001 - convert binding errors to action result
            self.get_logger().error(f"MTC task failed: {exc}")
            if execution_started:
                with self._state_lock:
                    self._recovery_required = True
            goal_handle.abort()
            self._last_command_valid = False
            return self._result(
                False,
                MoveItErrorCodes.FAILURE,
                f"MTC task failed: {exc}",
                "task initialization",
                solution_count,
            )
        finally:
            with self._state_lock:
                self._active_task = None
                self._active_execution_goal = None
                self._busy = False
            if self._recovery_required:
                self._set_status(
                    ExtendedStatus.STATE_FAULT,
                    "MTC execution state is uncertain; reconcile the robot and "
                    "planning scene, then restart this node",
                    False,
                )
            elif self._hardware_faulted:
                self._motion_lease.release()
                self._set_status(
                    ExtendedStatus.STATE_FAULT,
                    "Robot hardware is faulted; MTC task server is locked out",
                    False,
                )
            elif self._last_command_valid:
                self._motion_lease.release()
                self._set_status(
                    ExtendedStatus.STATE_IDLE,
                    "MTC task server idle",
                    True,
                )
            else:
                self._motion_lease.release()
                self._set_status(
                    ExtendedStatus.STATE_IDLE,
                    "Last MTC task failed; inspect the action result",
                    False,
                )
            # Ensure plugin-backed planner instances owned by the task disappear
            # before rclcpp is shut down.
            task = None

    def _build_task(
        self,
        object_spec: ObjectSpec,
        destination: DestinationSpec,
    ):
        robot = self._config.robot

        sampling = core.PipelinePlanner(
            self._mtc_node,
            robot.planning_pipeline,
        )
        sampling.planner = robot.planner_id
        sampling.num_planning_attempts = robot.num_planning_attempts
        sampling.max_velocity_scaling_factor = robot.max_velocity_scaling_factor
        sampling.max_acceleration_scaling_factor = (
            robot.max_acceleration_scaling_factor
        )

        cartesian = core.CartesianPath()
        cartesian.step_size = robot.cartesian_step_size
        cartesian.jump_threshold = robot.cartesian_jump_threshold
        cartesian.min_fraction = robot.cartesian_min_fraction
        cartesian.max_velocity_scaling_factor = robot.max_velocity_scaling_factor
        cartesian.max_acceleration_scaling_factor = (
            robot.max_acceleration_scaling_factor
        )

        gripper = core.JointInterpolationPlanner()
        gripper.max_velocity_scaling_factor = robot.max_velocity_scaling_factor
        gripper.max_acceleration_scaling_factor = (
            robot.max_acceleration_scaling_factor
        )

        task = core.Task()
        task.name = f"pick {object_spec.object_id} and place at {destination.destination_id}"
        task.timeout = self._planning_timeout
        task.loadRobotModel(self._mtc_node)
        task.enableIntrospection(True)

        current = stages.CurrentState("current state")
        current.timeout = robot.stage_timeout
        task.add(current)

        add_static_scene = stages.ModifyPlanningScene(
            "add static collision scene"
        )
        for scene_object in self._scene_objects:
            add_static_scene.addObject(
                self._static_collision_object(scene_object)
            )
        task.add(add_static_scene)

        add_object = stages.ModifyPlanningScene("add grasp object")
        add_object.addObject(self._collision_object(object_spec))
        task.add(add_object)

        open_hand = self._gripper_stage(
            "open gripper",
            gripper,
            robot.gripper_open_position,
        )
        task.add(open_hand)
        initial_state = task["open gripper"]

        connect_pick = stages.Connect(
            "move to pre-grasp",
            [(robot.arm_group, sampling)],
        )
        connect_pick.timeout = robot.stage_timeout
        connect_pick.setCostTerm(core.PathLength())
        task.add(connect_pick)

        pick = core.SerialContainer("pick object")

        approach = self._cartesian_stage(
            "approach object",
            cartesian,
            object_spec.approach,
        )
        pick.add(approach)

        grasp_generator = stages.GeneratePose("generate calibrated grasp pose")
        grasp_generator.pose = self._pose(object_spec.grasp_pose)
        grasp_generator.setMonitoredStage(initial_state)
        grasp_ik = self._compute_ik("compute grasp IK", grasp_generator)
        pick.add(grasp_ik)

        allow_touch = stages.ModifyPlanningScene(
            "allow object collision with gripper"
        )
        allow_touch.allowCollisions(
            object_spec.object_id,
            list(robot.touch_links),
            True,
        )
        pick.add(allow_touch)

        pick.add(
            self._gripper_stage(
                "close gripper",
                gripper,
                robot.gripper_closed_position,
            )
        )

        attach = stages.ModifyPlanningScene("attach object")
        attach.attachObject(object_spec.object_id, robot.attach_link)
        pick.add(attach)

        if object_spec.support_surface:
            allow_source_surface = stages.ModifyPlanningScene(
                "allow object collision with source support"
            )
            allow_source_surface.allowCollisions(
                object_spec.object_id,
                object_spec.support_surface,
                True,
            )
            pick.add(allow_source_surface)

        pick.add(self._cartesian_stage("lift object", cartesian, object_spec.lift))

        if object_spec.support_surface:
            forbid_source_surface = stages.ModifyPlanningScene(
                "restore source support collision checking"
            )
            forbid_source_surface.allowCollisions(
                object_spec.object_id,
                object_spec.support_surface,
                False,
            )
            pick.add(forbid_source_surface)

        task.add(pick)
        completed_pick = task["pick object"]

        connect_place = stages.Connect(
            "transport object",
            [(robot.arm_group, sampling)],
        )
        connect_place.timeout = robot.stage_timeout
        connect_place.setCostTerm(core.PathLength())
        task.add(connect_place)

        place = core.SerialContainer("place object")

        place.add(self._cartesian_stage("lower object", cartesian, destination.lower))

        place_generator = stages.GeneratePose("generate calibrated place pose")
        place_generator.pose = self._pose(destination.tool_pose)
        place_generator.setMonitoredStage(completed_pick)
        place_ik = self._compute_ik("compute place IK", place_generator)
        place.add(place_ik)

        place.add(
            self._gripper_stage(
                "open gripper at destination",
                gripper,
                robot.gripper_open_position,
            )
        )

        forbid_touch = stages.ModifyPlanningScene(
            "restore gripper collision checking"
        )
        forbid_touch.allowCollisions(
            object_spec.object_id,
            list(robot.touch_links),
            False,
        )
        place.add(forbid_touch)

        detach = stages.ModifyPlanningScene("detach object")
        detach.detachObject(object_spec.object_id, robot.attach_link)
        place.add(detach)

        place.add(
            self._cartesian_stage(
                "retreat after place",
                cartesian,
                destination.retreat,
            )
        )

        task.add(place)
        return task

    def _gripper_stage(self, name, planner, position):
        stage = stages.MoveTo(name, planner)
        stage.group = self._config.robot.gripper_group
        stage.timeout = self._config.robot.stage_timeout
        stage.setGoal({self._config.robot.gripper_joint: position})
        return stage

    def _cartesian_stage(self, name, planner, motion: MotionSpec):
        stage = stages.MoveRelative(name, planner)
        stage.group = self._config.robot.arm_group
        stage.ik_frame = self._ik_frame()
        stage.min_distance = motion.min_distance
        stage.max_distance = motion.max_distance
        stage.timeout = self._config.robot.stage_timeout
        stage.setDirection(self._direction(motion))
        return stage

    def _compute_ik(self, name, generator):
        robot = self._config.robot
        wrapper = stages.ComputeIK(name, generator)
        wrapper.group = robot.arm_group
        wrapper.ik_frame = self._ik_frame()
        wrapper.max_ik_solutions = robot.max_ik_solutions
        wrapper.min_solution_distance = robot.min_ik_solution_distance
        wrapper.timeout = robot.stage_timeout
        wrapper.properties.configureInitFrom(
            core.Stage.PropertyInitializerSource.INTERFACE,
            ["target_pose"],
        )
        return wrapper

    def _ik_frame(self):
        pose = PoseStamped()
        pose.header.frame_id = self._config.robot.ik_frame
        pose.pose.orientation.w = 1.0
        return pose

    @staticmethod
    def _pose(spec: PoseSpec):
        pose = PoseStamped()
        pose.header.frame_id = spec.frame_id
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
            spec.position
        )
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = spec.orientation
        return pose

    @staticmethod
    def _direction(spec: MotionSpec):
        direction = Vector3Stamped()
        direction.header.frame_id = spec.frame_id
        (
            direction.vector.x,
            direction.vector.y,
            direction.vector.z,
        ) = spec.direction
        return direction

    def _collision_object(self, spec: ObjectSpec):
        collision_object = CollisionObject()
        collision_object.id = spec.object_id
        collision_object.header.frame_id = spec.collision_pose.frame_id

        primitive = SolidPrimitive()
        primitive.type = {
            "box": SolidPrimitive.BOX,
            "sphere": SolidPrimitive.SPHERE,
            "cylinder": SolidPrimitive.CYLINDER,
            "cone": SolidPrimitive.CONE,
        }[spec.shape.kind]
        primitive.dimensions = list(spec.shape.dimensions)

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(self._pose(spec.collision_pose).pose)
        collision_object.operation = CollisionObject.ADD
        return collision_object

    def _static_collision_object(self, spec: SceneObjectSpec):
        collision_object = CollisionObject()
        collision_object.id = spec.object_id
        collision_object.header.frame_id = spec.pose.frame_id

        primitive = SolidPrimitive()
        primitive.type = {
            "box": SolidPrimitive.BOX,
            "sphere": SolidPrimitive.SPHERE,
            "cylinder": SolidPrimitive.CYLINDER,
            "cone": SolidPrimitive.CONE,
        }[spec.shape.kind]
        primitive.dimensions = list(spec.shape.dimensions)

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(self._pose(spec.pose).pose)
        collision_object.operation = CollisionObject.ADD
        return collision_object

    def _find_failure(self, task):
        candidates: list[tuple[str, str]] = []

        def visit(stages_container: Iterable):
            for stage in stages_container:
                try:
                    if len(stage.failures):
                        comment = stage.failures[0].comment
                        candidates.append((stage.name, comment))
                except (AttributeError, IndexError, TypeError):
                    pass
                try:
                    visit(stage)
                except TypeError:
                    pass

        visit(task)
        return candidates[-1] if candidates else ("task", "")

    def destroy_node(self):
        with self._state_lock:
            active_task = self._active_task
            execution_goal = self._active_execution_goal
        if active_task is not None:
            active_task.preempt()
        if execution_goal is not None:
            execution_goal.cancel_goal_async()
        self._action_server.destroy()
        self._active_task = None
        self._active_execution_goal = None
        self._motion_lease.release()
        self._mtc_node = None
        return super().destroy_node()


def main(args=None):
    if _MTC_IMPORT_ERROR is not None:
        raise RuntimeError(
            "Cannot start mtc_task_node because its Python bindings could not be "
            f"imported: {_MTC_IMPORT_ERROR}"
        )

    rclcpp.init(args)
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = MtcTaskNode()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        rclcpp.shutdown()


if __name__ == "__main__":
    main()
