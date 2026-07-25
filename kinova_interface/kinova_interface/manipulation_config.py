"""Strict, ROS-independent configuration for MTC pick-and-place tasks."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
from typing import Any, Mapping, Sequence


class ConfigurationError(ValueError):
    """Raised when a manipulation configuration is unsafe or incomplete."""


@dataclass(frozen=True)
class PoseSpec:
    frame_id: str
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]


@dataclass(frozen=True)
class MotionSpec:
    frame_id: str
    direction: tuple[float, float, float]
    min_distance: float
    max_distance: float


@dataclass(frozen=True)
class ShapeSpec:
    kind: str
    dimensions: tuple[float, ...]


@dataclass(frozen=True)
class SceneObjectSpec:
    object_id: str
    description: str
    shape: ShapeSpec
    pose: PoseSpec


@dataclass(frozen=True)
class RobotSpec:
    base_frame: str
    arm_group: str
    home_joint_positions: tuple[tuple[str, float], ...]
    gripper_group: str
    ik_frame: str
    attach_link: str
    gripper_joint: str
    touch_links: tuple[str, ...]
    gripper_min_position: float
    gripper_max_position: float
    gripper_open_position: float
    gripper_closed_position: float
    planning_pipeline: str
    planner_id: str
    num_planning_attempts: int
    max_solutions: int
    max_ik_solutions: int
    min_ik_solution_distance: float
    stage_timeout: float
    cartesian_step_size: float
    cartesian_jump_threshold: float
    cartesian_min_fraction: float
    max_velocity_scaling_factor: float
    max_acceleration_scaling_factor: float


@dataclass(frozen=True)
class ObjectSpec:
    object_id: str
    shape: ShapeSpec
    collision_pose: PoseSpec
    pregrasp_pose: PoseSpec
    grasp_pose: PoseSpec
    approach: MotionSpec
    lift: MotionSpec
    support_surface: str | None


@dataclass(frozen=True)
class DestinationSpec:
    destination_id: str
    tool_pose: PoseSpec
    lower: MotionSpec
    retreat: MotionSpec


@dataclass(frozen=True)
class ManipulationConfig:
    commissioned: bool
    robot: RobotSpec
    objects: Mapping[str, ObjectSpec]
    destinations: Mapping[str, DestinationSpec]

    def resolve(self, object_id: str, destination_id: str) -> tuple[ObjectSpec, DestinationSpec]:
        try:
            object_spec = self.objects[object_id]
        except KeyError as exc:
            known = ", ".join(sorted(self.objects))
            raise ConfigurationError(
                f"Unknown object_id '{object_id}'. Configured objects: {known}"
            ) from exc
        try:
            destination_spec = self.destinations[destination_id]
        except KeyError as exc:
            known = ", ".join(sorted(self.destinations))
            raise ConfigurationError(
                f"Unknown destination_id '{destination_id}'. Configured destinations: {known}"
            ) from exc
        return object_spec, destination_spec


_ID_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")
_SHAPE_DIMENSIONS = {"box": 3, "sphere": 1, "cylinder": 2, "cone": 2}
_SHAPE_CODES = {1: "box", 2: "sphere", 3: "cylinder", 4: "cone"}


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ConfigurationError(f"{path} must be a JSON object")
    return value


def _required(data: Mapping[str, Any], key: str, path: str) -> Any:
    if key not in data:
        raise ConfigurationError(f"{path}.{key} is required")
    return data[key]


def _reject_unknown(
    data: Mapping[str, Any],
    allowed: set[str],
    path: str,
) -> None:
    unknown = set(data).difference(allowed)
    if unknown:
        raise ConfigurationError(
            f"{path} contains unsupported field(s): {', '.join(sorted(unknown))}"
        )


def _boolean(value: Any, path: str) -> bool:
    if not isinstance(value, bool):
        raise ConfigurationError(f"{path} must be true or false")
    return value


def _string(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ConfigurationError(f"{path} must be a non-empty string")
    return value.strip()


def _identifier(value: Any, path: str) -> str:
    result = _string(value, path)
    if not _ID_PATTERN.fullmatch(result):
        raise ConfigurationError(
            f"{path} must start with a letter and contain only letters, digits, and underscores"
        )
    return result


def _number(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigurationError(f"{path} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ConfigurationError(f"{path} must be a finite number")
    return result


def _integer(value: Any, path: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigurationError(f"{path} must be an integer")
    if not minimum <= value <= maximum:
        raise ConfigurationError(f"{path} must be between {minimum} and {maximum}")
    return value


def _vector(value: Any, path: str, length: int) -> tuple[float, ...]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence) or len(value) != length:
        raise ConfigurationError(f"{path} must contain exactly {length} numbers")
    return tuple(_number(item, f"{path}[{index}]") for index, item in enumerate(value))


def _frame(value: Any, path: str) -> str:
    frame = _string(value, path)
    if frame.startswith("/"):
        raise ConfigurationError(f"{path} must not start with '/'")
    return frame


def _pose(value: Any, path: str) -> PoseSpec:
    data = _mapping(value, path)
    _reject_unknown(data, {"frame_id", "position", "orientation"}, path)
    position = _vector(_required(data, "position", path), f"{path}.position", 3)
    orientation = _vector(_required(data, "orientation", path), f"{path}.orientation", 4)
    norm = math.sqrt(sum(component * component for component in orientation))
    if norm < 1e-9:
        raise ConfigurationError(f"{path}.orientation cannot be the zero quaternion")
    if abs(norm - 1.0) > 1e-3:
        raise ConfigurationError(
            f"{path}.orientation must be normalized (norm is {norm:.6f})"
        )
    normalized = tuple(component / norm for component in orientation)
    return PoseSpec(
        frame_id=_frame(_required(data, "frame_id", path), f"{path}.frame_id"),
        position=(position[0], position[1], position[2]),
        orientation=(normalized[0], normalized[1], normalized[2], normalized[3]),
    )


def _motion(value: Any, path: str) -> MotionSpec:
    data = _mapping(value, path)
    _reject_unknown(
        data,
        {"frame_id", "direction", "min_distance", "max_distance"},
        path,
    )
    direction = _vector(_required(data, "direction", path), f"{path}.direction", 3)
    norm = math.sqrt(sum(component * component for component in direction))
    if norm < 1e-9:
        raise ConfigurationError(f"{path}.direction cannot be a zero vector")
    min_distance = _number(_required(data, "min_distance", path), f"{path}.min_distance")
    max_distance = _number(_required(data, "max_distance", path), f"{path}.max_distance")
    if min_distance <= 0.0 or max_distance <= 0.0:
        raise ConfigurationError(f"{path} distances must be positive")
    if min_distance > max_distance:
        raise ConfigurationError(f"{path}.min_distance cannot exceed max_distance")
    if max_distance > 1.0:
        raise ConfigurationError(f"{path}.max_distance cannot exceed 1 metre")
    return MotionSpec(
        frame_id=_frame(_required(data, "frame_id", path), f"{path}.frame_id"),
        direction=tuple(component / norm for component in direction),
        min_distance=min_distance,
        max_distance=max_distance,
    )


def _tool_z_axis(orientation: tuple[float, float, float, float]):
    """Return the tool-frame +Z axis expressed in the pose parent frame."""

    x, y, z, w = orientation
    return (
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
        1.0 - 2.0 * (x * x + y * y),
    )


def _locked_approach(
    pregrasp: PoseSpec,
    grasp: PoseSpec,
    base_frame: str,
    path: str,
) -> MotionSpec:
    """Validate and derive one exact, vertically downward approach."""

    if pregrasp.frame_id != base_frame or grasp.frame_id != base_frame:
        raise ConfigurationError(
            f"{path} pregrasp_pose and grasp_pose must use robot.base_frame "
            f"'{base_frame}'"
        )

    orientation_dot = sum(
        first * second
        for first, second in zip(pregrasp.orientation, grasp.orientation)
    )
    if abs(abs(orientation_dot) - 1.0) > 1e-4:
        raise ConfigurationError(
            f"{path} pregrasp_pose and grasp_pose orientations must be identical"
        )

    for label, pose in (("pregrasp_pose", pregrasp), ("grasp_pose", grasp)):
        tool_z = _tool_z_axis(pose.orientation)
        if tool_z[2] > -0.999 or abs(tool_z[0]) > 0.045 or abs(tool_z[1]) > 0.045:
            raise ConfigurationError(
                f"{path}.{label} must point tool_frame +Z directly toward "
                "the table (-Z in robot.base_frame)"
            )

    dx = grasp.position[0] - pregrasp.position[0]
    dy = grasp.position[1] - pregrasp.position[1]
    dz = grasp.position[2] - pregrasp.position[2]
    if abs(dx) > 1e-4 or abs(dy) > 1e-4:
        raise ConfigurationError(
            f"{path} grasp_pose must be directly below pregrasp_pose "
            "with identical X and Y"
        )
    distance = -dz
    if not 0.02 <= distance <= 0.25:
        raise ConfigurationError(
            f"{path} vertical pregrasp-to-grasp distance must be between "
            "0.02 and 0.25 metres"
        )

    return MotionSpec(
        frame_id=base_frame,
        direction=(0.0, 0.0, -1.0),
        min_distance=distance,
        max_distance=distance,
    )


def _shape(value: Any, path: str) -> ShapeSpec:
    data = _mapping(value, path)
    _reject_unknown(data, {"type", "dimensions"}, path)
    kind = _string(_required(data, "type", path), f"{path}.type").lower()
    if kind not in _SHAPE_DIMENSIONS:
        supported = ", ".join(sorted(_SHAPE_DIMENSIONS))
        raise ConfigurationError(f"{path}.type must be one of: {supported}")
    dimensions = _vector(
        _required(data, "dimensions", path),
        f"{path}.dimensions",
        _SHAPE_DIMENSIONS[kind],
    )
    if any(dimension <= 0.0 for dimension in dimensions):
        raise ConfigurationError(f"{path}.dimensions must all be positive")
    if any(dimension > 1.0 for dimension in dimensions):
        raise ConfigurationError(f"{path}.dimensions cannot exceed 1 metre")
    return ShapeSpec(kind=kind, dimensions=dimensions)


def _scale(value: Any, path: str) -> float:
    result = _number(value, path)
    if not 0.0 < result <= 1.0:
        raise ConfigurationError(f"{path} must be in the interval (0, 1]")
    return result


def _robot(value: Any) -> RobotSpec:
    path = "robot"
    data = _mapping(value, path)
    _reject_unknown(
        data,
        {
            "base_frame",
            "arm_group",
            "home_joint_positions",
            "gripper_group",
            "ik_frame",
            "attach_link",
            "gripper_joint",
            "touch_links",
            "gripper_min_position",
            "gripper_max_position",
            "gripper_open_position",
            "gripper_closed_position",
            "planning_pipeline",
            "planner_id",
            "num_planning_attempts",
            "max_solutions",
            "max_ik_solutions",
            "min_ik_solution_distance",
            "stage_timeout",
            "cartesian_step_size",
            "cartesian_jump_threshold",
            "cartesian_min_fraction",
            "max_velocity_scaling_factor",
            "max_acceleration_scaling_factor",
        },
        path,
    )
    links_raw = _required(data, "touch_links", path)
    if isinstance(links_raw, (str, bytes)) or not isinstance(links_raw, Sequence):
        raise ConfigurationError("robot.touch_links must be a non-empty list")
    touch_links = tuple(
        _identifier(link, f"robot.touch_links[{index}]")
        for index, link in enumerate(links_raw)
    )
    if not touch_links or len(set(touch_links)) != len(touch_links):
        raise ConfigurationError("robot.touch_links must contain unique link names")

    home_raw = _mapping(
        _required(data, "home_joint_positions", path),
        "robot.home_joint_positions",
    )
    expected_home_joints = {f"joint_{index}" for index in range(1, 7)}
    if set(home_raw) != expected_home_joints:
        raise ConfigurationError(
            "robot.home_joint_positions must define exactly joint_1 through joint_6"
        )
    home_joint_positions = tuple(
        (
            _identifier(joint, f"robot.home_joint_positions key {joint!r}"),
            _number(value, f"robot.home_joint_positions.{joint}"),
        )
        for joint, value in sorted(home_raw.items())
    )

    minimum = _number(
        _required(data, "gripper_min_position", path), "robot.gripper_min_position"
    )
    maximum = _number(
        _required(data, "gripper_max_position", path), "robot.gripper_max_position"
    )
    opened = _number(
        _required(data, "gripper_open_position", path), "robot.gripper_open_position"
    )
    closed = _number(
        _required(data, "gripper_closed_position", path), "robot.gripper_closed_position"
    )
    if minimum >= maximum:
        raise ConfigurationError(
            "robot.gripper_min_position must be less than gripper_max_position"
        )
    if not minimum <= opened <= maximum or not minimum <= closed <= maximum:
        raise ConfigurationError(
            "robot gripper open/closed positions must lie inside the configured joint limits"
        )
    if opened >= closed:
        raise ConfigurationError(
            "Kinova Gen3 Lite requires gripper_open_position < gripper_closed_position"
        )

    stage_timeout = _number(
        _required(data, "stage_timeout", path), "robot.stage_timeout"
    )
    if not 0.0 < stage_timeout <= 300.0:
        raise ConfigurationError("robot.stage_timeout must be in the interval (0, 300]")

    cartesian_step = _number(
        _required(data, "cartesian_step_size", path), "robot.cartesian_step_size"
    )
    if not 0.0001 <= cartesian_step <= 0.05:
        raise ConfigurationError(
            "robot.cartesian_step_size must be between 0.0001 and 0.05 metres"
        )
    jump_threshold = _number(
        _required(data, "cartesian_jump_threshold", path),
        "robot.cartesian_jump_threshold",
    )
    if jump_threshold < 0.0:
        raise ConfigurationError("robot.cartesian_jump_threshold cannot be negative")

    min_ik_distance = _number(
        _required(data, "min_ik_solution_distance", path),
        "robot.min_ik_solution_distance",
    )
    if min_ik_distance < 0.0:
        raise ConfigurationError("robot.min_ik_solution_distance cannot be negative")

    return RobotSpec(
        base_frame=_frame(_required(data, "base_frame", path), "robot.base_frame"),
        arm_group=_identifier(_required(data, "arm_group", path), "robot.arm_group"),
        home_joint_positions=home_joint_positions,
        gripper_group=_identifier(
            _required(data, "gripper_group", path), "robot.gripper_group"
        ),
        ik_frame=_identifier(_required(data, "ik_frame", path), "robot.ik_frame"),
        attach_link=_identifier(
            _required(data, "attach_link", path), "robot.attach_link"
        ),
        gripper_joint=_identifier(
            _required(data, "gripper_joint", path), "robot.gripper_joint"
        ),
        touch_links=touch_links,
        gripper_min_position=minimum,
        gripper_max_position=maximum,
        gripper_open_position=opened,
        gripper_closed_position=closed,
        planning_pipeline=_identifier(
            _required(data, "planning_pipeline", path), "robot.planning_pipeline"
        ),
        planner_id=_identifier(_required(data, "planner_id", path), "robot.planner_id"),
        num_planning_attempts=_integer(
            _required(data, "num_planning_attempts", path),
            "robot.num_planning_attempts",
            1,
            100,
        ),
        max_solutions=_integer(
            _required(data, "max_solutions", path), "robot.max_solutions", 1, 100
        ),
        max_ik_solutions=_integer(
            _required(data, "max_ik_solutions", path),
            "robot.max_ik_solutions",
            1,
            100,
        ),
        min_ik_solution_distance=min_ik_distance,
        stage_timeout=stage_timeout,
        cartesian_step_size=cartesian_step,
        cartesian_jump_threshold=jump_threshold,
        cartesian_min_fraction=_scale(
            _required(data, "cartesian_min_fraction", path),
            "robot.cartesian_min_fraction",
        ),
        max_velocity_scaling_factor=_scale(
            _required(data, "max_velocity_scaling_factor", path),
            "robot.max_velocity_scaling_factor",
        ),
        max_acceleration_scaling_factor=_scale(
            _required(data, "max_acceleration_scaling_factor", path),
            "robot.max_acceleration_scaling_factor",
        ),
    )


def _optional_surface(data: Mapping[str, Any], path: str) -> str | None:
    value = data.get("support_surface")
    if value is None:
        return None
    return _identifier(value, f"{path}.support_surface")


def load_manipulation_config(config_path: str | Path) -> ManipulationConfig:
    """Load and validate the complete manipulation configuration."""

    path = Path(config_path)
    try:
        with path.open("r", encoding="utf-8") as stream:
            raw = json.load(stream)
    except FileNotFoundError as exc:
        raise ConfigurationError(f"Manipulation config does not exist: {path}") from exc
    except json.JSONDecodeError as exc:
        raise ConfigurationError(
            f"Invalid JSON in {path} at line {exc.lineno}, column {exc.colno}: {exc.msg}"
        ) from exc
    except OSError as exc:
        raise ConfigurationError(f"Cannot read manipulation config {path}: {exc}") from exc

    root = _mapping(raw, "root")
    _reject_unknown(
        root,
        {"schema_version", "commissioned", "robot", "objects", "destinations"},
        "root",
    )
    version = _required(root, "schema_version", "root")
    if version != 1:
        raise ConfigurationError(
            f"root.schema_version must be 1; received {version!r}"
        )
    commissioned = _boolean(
        _required(root, "commissioned", "root"),
        "root.commissioned",
    )
    robot = _robot(_required(root, "robot", "root"))

    objects_raw = _mapping(_required(root, "objects", "root"), "objects")
    if not objects_raw:
        raise ConfigurationError("objects must contain at least one configured object")
    objects: dict[str, ObjectSpec] = {}
    for raw_id, raw_object in objects_raw.items():
        object_id = _identifier(raw_id, "objects key")
        object_path = f"objects.{object_id}"
        data = _mapping(raw_object, object_path)
        _reject_unknown(
            data,
            {
                "shape",
                "collision_pose",
                "pregrasp_pose",
                "grasp_pose",
                "lift",
                "support_surface",
            },
            object_path,
        )
        pregrasp_pose = _pose(
            _required(data, "pregrasp_pose", object_path),
            f"{object_path}.pregrasp_pose",
        )
        grasp_pose = _pose(
            _required(data, "grasp_pose", object_path),
            f"{object_path}.grasp_pose",
        )
        objects[object_id] = ObjectSpec(
            object_id=object_id,
            shape=_shape(_required(data, "shape", object_path), f"{object_path}.shape"),
            collision_pose=_pose(
                _required(data, "collision_pose", object_path),
                f"{object_path}.collision_pose",
            ),
            pregrasp_pose=pregrasp_pose,
            grasp_pose=grasp_pose,
            approach=_locked_approach(
                pregrasp_pose,
                grasp_pose,
                robot.base_frame,
                object_path,
            ),
            lift=_motion(_required(data, "lift", object_path), f"{object_path}.lift"),
            support_surface=_optional_surface(data, object_path),
        )

    destinations_raw = _mapping(
        _required(root, "destinations", "root"), "destinations"
    )
    if not destinations_raw:
        raise ConfigurationError(
            "destinations must contain at least one configured destination"
        )
    destinations: dict[str, DestinationSpec] = {}
    for raw_id, raw_destination in destinations_raw.items():
        destination_id = _identifier(raw_id, "destinations key")
        destination_path = f"destinations.{destination_id}"
        data = _mapping(raw_destination, destination_path)
        _reject_unknown(
            data,
            {"tool_pose", "lower", "retreat"},
            destination_path,
        )
        destinations[destination_id] = DestinationSpec(
            destination_id=destination_id,
            tool_pose=_pose(
                _required(data, "tool_pose", destination_path),
                f"{destination_path}.tool_pose",
            ),
            lower=_motion(
                _required(data, "lower", destination_path),
                f"{destination_path}.lower",
            ),
            retreat=_motion(
                _required(data, "retreat", destination_path),
                f"{destination_path}.retreat",
            ),
        )

    return ManipulationConfig(
        commissioned=commissioned,
        robot=robot,
        objects=objects,
        destinations=destinations,
    )


def load_scene_objects(config_path: str | Path) -> tuple[SceneObjectSpec, ...]:
    """Load the repository's static planning-scene objects strictly."""

    path = Path(config_path)
    try:
        with path.open("r", encoding="utf-8") as stream:
            raw = json.load(stream)
    except FileNotFoundError as exc:
        raise ConfigurationError(f"Scene object config does not exist: {path}") from exc
    except json.JSONDecodeError as exc:
        raise ConfigurationError(
            f"Invalid JSON in {path} at line {exc.lineno}, column {exc.colno}: {exc.msg}"
        ) from exc
    except OSError as exc:
        raise ConfigurationError(f"Cannot read scene object config {path}: {exc}") from exc

    if isinstance(raw, (str, bytes)) or not isinstance(raw, Sequence) or not raw:
        raise ConfigurationError("obstacles root must be a non-empty JSON array")

    objects: list[SceneObjectSpec] = []
    seen: set[str] = set()
    for index, raw_object in enumerate(raw):
        object_path = f"obstacles[{index}]"
        data = _mapping(raw_object, object_path)
        _reject_unknown(
            data,
            {"id", "description", "shape", "dimensions", "position", "frame_id"},
            object_path,
        )
        object_id = _identifier(
            _required(data, "id", object_path), f"{object_path}.id"
        )
        if object_id in seen:
            raise ConfigurationError(f"Duplicate scene object ID '{object_id}'")
        seen.add(object_id)

        shape_code = _required(data, "shape", object_path)
        if (
            isinstance(shape_code, bool)
            or not isinstance(shape_code, int)
            or shape_code not in _SHAPE_CODES
        ):
            raise ConfigurationError(
                f"{object_path}.shape must be a SolidPrimitive code from 1 through 4"
            )
        kind = _SHAPE_CODES[shape_code]
        dimensions = _vector(
            _required(data, "dimensions", object_path),
            f"{object_path}.dimensions",
            _SHAPE_DIMENSIONS[kind],
        )
        if any(dimension <= 0.0 or dimension > 10.0 for dimension in dimensions):
            raise ConfigurationError(
                f"{object_path}.dimensions must be positive and at most 10 metres"
            )

        position_data = _mapping(
            _required(data, "position", object_path),
            f"{object_path}.position",
        )
        _reject_unknown(position_data, {"x", "y", "z"}, f"{object_path}.position")
        position = tuple(
            _number(
                _required(position_data, axis, f"{object_path}.position"),
                f"{object_path}.position.{axis}",
            )
            for axis in ("x", "y", "z")
        )
        description = data.get("description", object_id)
        description = _string(description, f"{object_path}.description")
        frame_id = _frame(
            data.get("frame_id", "base_link"),
            f"{object_path}.frame_id",
        )

        objects.append(
            SceneObjectSpec(
                object_id=object_id,
                description=description,
                shape=ShapeSpec(kind=kind, dimensions=dimensions),
                pose=PoseSpec(
                    frame_id=frame_id,
                    position=(position[0], position[1], position[2]),
                    orientation=(0.0, 0.0, 0.0, 1.0),
                ),
            )
        )

    return tuple(objects)
