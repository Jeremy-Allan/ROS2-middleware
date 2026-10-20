import pytest
import rclpy

from unittest.mock import MagicMock, patch

from kinova_interface.nodes.telemetry_node import TelemetryNode

from kinova_interfaces.msg import ExtendedStatus, SystemSummary
from std_srvs.srv import Trigger
from example_interfaces.srv import Trigger as ExampleTrigger


"""
Test with: 
pytest src/ROS2-middleware/kinova_interface/test/test_telemetry_node.py -v
"""

# Fixtures

@pytest.fixture
def node(ros_context):
    """Create a TelemetryNode for testing."""

    with patch.object(TelemetryNode, "create_timer") as mock_timer:
        mock_timer.return_value = MagicMock()

        node = TelemetryNode()

    yield node
    node.destroy_node()


# Node Initialisation
def test_node_initialises(node):
    """Test that the telemetry node creates its ROS interfaces."""

    assert node.sub is not None
    assert node.pub is not None
    assert node.timer is not None

    assert node.reset_client is not None
    assert node.reset_service is not None

    assert node.tracked_nodes == {}


# handle_reset_fault()
def test_handle_reset_fault_success(node):
    """Test successful forwarding of a fault reset request."""

    node.reset_client.wait_for_service = MagicMock(
        return_value=True
    )

    service_result = ExampleTrigger.Response()
    service_result.success = True
    service_result.message = "Faults cleared"

    node.reset_client.call = MagicMock(
        return_value=service_result
    )

    request = Trigger.Request()
    response = Trigger.Response()

    result = node.handle_reset_fault(
        request,
        response
    )

    assert result.success is True
    assert result.message == "Faults cleared"

    node.reset_client.call.assert_called_once()


def test_handle_reset_fault_service_unavailable(node):
    """Test fault reset when hardware service is unavailable."""

    node.reset_client.wait_for_service = MagicMock(
        return_value=False
    )

    request = Trigger.Request()
    response = Trigger.Response()

    result = node.handle_reset_fault(
        request,
        response
    )

    assert result.success is False
    assert result.message == "Hardware driver service not ready."


# report_callback()
def test_report_callback_tracks_node(node):
    """Test that an incoming node status is stored."""

    msg = ExtendedStatus()
    msg.node_name = "test_node"
    msg.state = ExtendedStatus.STATE_IDLE
    msg.status_message = "Ready"

    node.report_callback(msg)

    assert "test_node" in node.tracked_nodes
    assert node.tracked_nodes["test_node"]["msg"] == msg
    assert node.tracked_nodes["test_node"]["last_seen"] is not None


def test_report_callback_updates_existing_node(node):
    """Test that an existing node's status is updated."""

    first_msg = ExtendedStatus()
    first_msg.node_name = "test_node"
    first_msg.state = ExtendedStatus.STATE_IDLE
    first_msg.status_message = "Ready"

    node.report_callback(first_msg)

    second_msg = ExtendedStatus()
    second_msg.node_name = "test_node"
    second_msg.state = ExtendedStatus.STATE_BUSY
    second_msg.status_message = "Working"

    node.report_callback(second_msg)

    assert node.tracked_nodes["test_node"]["msg"] == second_msg
    assert (
        node.tracked_nodes["test_node"]["msg"].state
        == ExtendedStatus.STATE_BUSY
    )


def test_report_callback_fault_state(node):
    """Test that a node entering FAULT is recorded."""

    msg = ExtendedStatus()
    msg.node_name = "test_node"
    msg.state = ExtendedStatus.STATE_FAULT
    msg.status_message = "Something went wrong"

    node.report_callback(msg)

    assert node.tracked_nodes["test_node"]["msg"].state == (
        ExtendedStatus.STATE_FAULT
    )

# aggregate_and_publish()
def test_aggregate_no_nodes(node):
    """No nodes should result in SYSTEM_FAULT."""

    node.pub.publish = MagicMock()

    node.aggregate_and_publish()

    node.pub.publish.assert_called_once()

    summary = node.pub.publish.call_args[0][0]

    assert summary.summary_state == SystemSummary.SYSTEM_FAULT
    assert len(summary.individual_states) == 0


def test_aggregate_ready(node):
    """All idle nodes should produce SYSTEM_READY."""

    msg = ExtendedStatus()
    msg.node_name = "node1"
    msg.state = ExtendedStatus.STATE_IDLE
    msg.status_message = "Ready"

    node.report_callback(msg)

    node.pub.publish = MagicMock()

    node.aggregate_and_publish()

    summary = node.pub.publish.call_args[0][0]

    assert summary.summary_state == SystemSummary.SYSTEM_READY
    assert len(summary.individual_states) == 1


def test_aggregate_busy(node):
    """A busy node should produce SYSTEM_BUSY."""

    msg = ExtendedStatus()
    msg.node_name = "node1"
    msg.state = ExtendedStatus.STATE_BUSY
    msg.status_message = "Working"

    node.report_callback(msg)

    node.pub.publish = MagicMock()

    node.aggregate_and_publish()

    summary = node.pub.publish.call_args[0][0]

    assert summary.summary_state == SystemSummary.SYSTEM_BUSY


def test_aggregate_fault(node):
    """A faulted node should produce SYSTEM_FAULT."""

    msg = ExtendedStatus()
    msg.node_name = "node1"
    msg.state = ExtendedStatus.STATE_FAULT
    msg.status_message = "Fault"

    node.report_callback(msg)

    node.pub.publish = MagicMock()

    node.aggregate_and_publish()

    summary = node.pub.publish.call_args[0][0]

    assert summary.summary_state == SystemSummary.SYSTEM_FAULT


def test_aggregate_fault_has_priority_over_busy(node):
    """FAULT should take priority over BUSY."""

    busy_msg = ExtendedStatus()
    busy_msg.node_name = "busy_node"
    busy_msg.state = ExtendedStatus.STATE_BUSY
    busy_msg.status_message = "Working"

    fault_msg = ExtendedStatus()
    fault_msg.node_name = "fault_node"
    fault_msg.state = ExtendedStatus.STATE_FAULT
    fault_msg.status_message = "Fault"

    node.report_callback(busy_msg)
    node.report_callback(fault_msg)

    node.pub.publish = MagicMock()

    node.aggregate_and_publish()

    summary = node.pub.publish.call_args[0][0]

    assert summary.summary_state == SystemSummary.SYSTEM_FAULT


def test_aggregate_stale_node(node):
    """A node with an old heartbeat should be marked as FAULT."""

    msg = ExtendedStatus()
    msg.node_name = "stale_node"
    msg.state = ExtendedStatus.STATE_IDLE
    msg.status_message = "Ready"

    node.report_callback(msg)

    # Make the heartbeat older than the 1.5 second timeout
    node.tracked_nodes["stale_node"]["last_seen"] = (
        node.get_clock().now()
        - node.heartbeat_timeout
        - rclpy.duration.Duration(seconds=1.0)
    )

    node.pub.publish = MagicMock()

    node.aggregate_and_publish()

    summary = node.pub.publish.call_args[0][0]

    assert summary.summary_state == SystemSummary.SYSTEM_FAULT

    stale_msg = summary.individual_states[0]

    assert stale_msg.state == ExtendedStatus.STATE_FAULT
    assert stale_msg.last_command_valid is False
    assert "heartbeat timed out" in stale_msg.status_message
