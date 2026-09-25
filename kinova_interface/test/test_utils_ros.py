from unittest.mock import MagicMock

from kinova_interface.utils.ros import call_service, wait_for_future


"""
Test with:
pytest src/ROS2-middleware/kinova_interface/test/test_utils_ros.py -v
"""


def _done_future(result):
    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = result
    return future


def test_wait_for_future_returns_result(ros_context):
    logger = MagicMock()
    assert wait_for_future(_done_future("ok"), "/svc", logger) == "ok"
    logger.error.assert_not_called()


def test_wait_for_future_times_out(ros_context):
    """A future that never completes returns None and logs the timeout."""
    future = MagicMock()
    future.done.return_value = False
    logger = MagicMock()

    assert wait_for_future(future, "/svc", logger, timeout_sec=0.05) is None
    logger.error.assert_called_once_with("Timed out waiting for /svc")


def test_call_service_success(ros_context):
    client = MagicMock()
    client.wait_for_service.return_value = True
    client.call_async.return_value = _done_future("response")
    request = object()

    assert call_service(client, request, "/svc", MagicMock()) == "response"
    client.call_async.assert_called_once_with(request)


def test_call_service_unavailable_skips_call(ros_context):
    """An unavailable service returns None without ever sending the request."""
    client = MagicMock()
    client.wait_for_service.return_value = False
    logger = MagicMock()

    assert call_service(client, object(), "/svc", logger, service_wait_sec=0.1) is None
    client.wait_for_service.assert_called_once_with(timeout_sec=0.1)
    client.call_async.assert_not_called()
    logger.error.assert_called_once_with("/svc service not available")
