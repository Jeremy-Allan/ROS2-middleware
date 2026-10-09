from pathlib import Path

import pytest
import rclpy

MOTION_SETTINGS = Path(__file__).parent.parent / 'data' / 'configs' / 'motion_settings.yaml'


@pytest.fixture(scope="session")
def ros_context():
    """
    Session-scoped rclpy context shared across all test files.
    Loads motion_settings.yaml like launch does, so nodes get its parameters.
    """
    already_initialised = rclpy.ok()

    if not already_initialised:
        rclpy.init(args=['--ros-args', '--params-file', str(MOTION_SETTINGS)])

    yield

    if not already_initialised and rclpy.ok():
        rclpy.shutdown()