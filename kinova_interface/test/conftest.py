from pathlib import Path

import pytest
import rclpy

GRASPING_PARAMS = Path(__file__).parent.parent / 'data' / 'configs' / 'grasping.yaml'


@pytest.fixture(scope="session")
def ros_context():
    """
    Session-scoped rclpy context shared across all test files.
    Loads grasping.yaml like launch does, so nodes get the grasping parameters.
    """
    already_initialised = rclpy.ok()

    if not already_initialised:
        rclpy.init(args=['--ros-args', '--params-file', str(GRASPING_PARAMS)])

    yield

    if not already_initialised and rclpy.ok():
        rclpy.shutdown()