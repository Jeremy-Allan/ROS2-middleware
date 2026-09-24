"""Small rclpy helpers shared by the nodes and actions."""
import time

import rclpy


def wait_for_future(future, service_name, logger, timeout_sec=10.0):
    """Safely wait for an async service call future to complete without
    deadlocking the executor (it's serviced by the node's other executor
    threads, so this just polls). Returns the result, or None on timeout."""
    start = time.time()
    while rclpy.ok() and not future.done():
        if time.time() - start > timeout_sec:
            logger.error(f"Timed out waiting for {service_name}")
            return None
        time.sleep(0.01)
    return future.result() if future.done() else None


def call_service(client, request, service_name, logger, timeout_sec=10.0, service_wait_sec=5.0):
    """Wait for `client`'s service, call it with `request`, and wait for the
    response. Returns the response, or None if the service isn't available
    or doesn't answer within `timeout_sec` (both logged as errors)."""
    if not client.wait_for_service(timeout_sec=service_wait_sec):
        logger.error(f"{service_name} service not available")
        return None
    return wait_for_future(client.call_async(request), service_name, logger, timeout_sec)
