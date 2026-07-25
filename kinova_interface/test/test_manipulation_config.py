import copy
import json
from pathlib import Path
import tempfile
import unittest

from kinova_interface.manipulation_config import (
    ConfigurationError,
    load_manipulation_config,
    load_scene_objects,
)


CONFIG_PATH = (
    Path(__file__).parents[1]
    / "data"
    / "configs"
    / "env"
    / "manipulation_objects.json"
)


class ManipulationConfigTest(unittest.TestCase):
    def raw_config(self):
        with CONFIG_PATH.open("r", encoding="utf-8") as stream:
            return json.load(stream)

    def write_config(self, directory, config):
        path = Path(directory) / "manipulation_objects.json"
        path.write_text(json.dumps(config), encoding="utf-8")
        return path

    def test_repository_config_is_valid_and_resolvable(self):
        config = load_manipulation_config(CONFIG_PATH)
        object_spec, destination = config.resolve("red_cube", "delivery_tray")

        self.assertTrue(config.commissioned)
        self.assertEqual(config.robot.arm_group, "arm")
        self.assertEqual(
            dict(config.robot.home_joint_positions),
            {
                "joint_1": 0.0,
                "joint_2": 0.0,
                "joint_3": 1.5708,
                "joint_4": 1.5708,
                "joint_5": 1.5708,
                "joint_6": 0.0,
            },
        )
        self.assertEqual(config.robot.gripper_group, "gripper")
        self.assertLessEqual(
            config.robot.gripper_closed_position,
            config.robot.gripper_max_position,
        )
        self.assertEqual(object_spec.shape.kind, "box")
        self.assertEqual(
            object_spec.pregrasp_pose.position,
            (-0.3255, -0.1235, 0.09),
        )
        self.assertEqual(object_spec.approach.direction, (0.0, 0.0, -1.0))
        self.assertAlmostEqual(object_spec.approach.min_distance, 0.08)
        self.assertEqual(
            object_spec.pregrasp_pose.orientation,
            object_spec.grasp_pose.orientation,
        )
        self.assertEqual(destination.retreat.direction, (0.0, 0.0, 1.0))

    def test_unknown_object_is_rejected_with_known_ids(self):
        config = load_manipulation_config(CONFIG_PATH)

        with self.assertRaisesRegex(
            ConfigurationError,
            "Configured objects: blue_cube, red_cube",
        ):
            config.resolve("green_cube", "delivery_tray")

    def test_static_scene_objects_are_valid(self):
        scene_path = CONFIG_PATH.with_name("obstacles.json")
        objects = load_scene_objects(scene_path)

        self.assertEqual([item.object_id for item in objects], ["table", "small_wall"])
        self.assertEqual(objects[0].shape.kind, "box")
        self.assertEqual(objects[0].pose.frame_id, "base_link")

    def test_unsafe_configuration_is_rejected(self):
        cases = [
            (
                lambda data: data["objects"]["red_cube"]["grasp_pose"].update(
                    {"orientation": [0.0, 0.0, 0.0, 0.0]}
                ),
                "zero quaternion",
            ),
            (
                lambda data: data["objects"]["red_cube"]["pregrasp_pose"].update(
                    {"position": [-0.30, -0.1235, 0.09]}
                ),
                "directly below pregrasp_pose",
            ),
            (
                lambda data: (
                    data["objects"]["red_cube"]["pregrasp_pose"].update(
                        {"orientation": [0.0, 0.0, 0.0, 1.0]}
                    ),
                    data["objects"]["red_cube"]["grasp_pose"].update(
                        {"orientation": [0.0, 0.0, 0.0, 1.0]}
                    ),
                ),
                "directly toward the table",
            ),
            (
                lambda data: data["robot"].update(
                    {"gripper_closed_position": 0.9}
                ),
                "inside the configured joint limits",
            ),
            (
                lambda data: data["objects"]["red_cube"]["shape"].update(
                    {"dimensions": [0.02, -0.02, 0.02]}
                ),
                "must all be positive",
            ),
            (
                lambda data: data["destinations"]["delivery_tray"].update(
                    {"support_surface": "table"}
                ),
                "unsupported field",
            ),
        ]

        with tempfile.TemporaryDirectory() as directory:
            for mutator, message in cases:
                with self.subTest(message=message):
                    data = copy.deepcopy(self.raw_config())
                    mutator(data)
                    path = self.write_config(directory, data)
                    with self.assertRaisesRegex(ConfigurationError, message):
                        load_manipulation_config(path)


if __name__ == "__main__":
    unittest.main()
