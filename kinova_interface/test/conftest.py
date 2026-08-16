import pytest
import rclpy


@pytest.fixture(scope="session")
def ros_context():
    """
    Session-scoped rclpy context shared across all test files.
    """
    already_initialised = rclpy.ok()

    if not already_initialised:
        rclpy.init()

    yield

    if not already_initialised and rclpy.ok():
        rclpy.shutdown()