import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from example_interfaces.srv import Trigger as ExampleTrigger
from kinova_interfaces.msg import ExtendedStatus, SystemSummary

# Imports for controller auto-activation workaround
import threading
import time
from controller_manager_msgs.srv import ConfigureController, SwitchController

class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')
        
        # Subscriber for individual node reports
        self.sub = self.create_subscription(
            ExtendedStatus,
            '/status/node_report',
            self.report_callback,
            10
        )
        
        # Publisher for aggregated system summary
        self.pub = self.create_publisher(
            SystemSummary,
            '/system/status',
            10
        )
        
        # Aggregation and publishing timer (0.5 seconds / 2Hz)
        self.timer = self.create_timer(0.5, self.aggregate_and_publish)
        
        # Registry to track status of each node
        self.tracked_nodes = {}
        
        # Stale heartbeat timeout (1.5 seconds)
        self.heartbeat_timeout = Duration(seconds=1.5)
        
        self.cb_group = ReentrantCallbackGroup()
        self.reset_client = self.create_client(
            ExampleTrigger,
            '/fault_controller/reset_fault',
            callback_group=self.cb_group
        )
        self.reset_service = self.create_service(
            Trigger,
            '/system/reset_fault',
            self.handle_reset_fault,
            callback_group=self.cb_group
        )
        
        # Self-healing controller manager clients
        self.activating_fault_controller = False
        self.configure_ctrl_client = self.create_client(
            ConfigureController,
            '/controller_manager/configure_controller',
            callback_group=self.cb_group
        )
        self.switch_ctrl_client = self.create_client(
            SwitchController,
            '/controller_manager/switch_controller',
            callback_group=self.cb_group
        )
        
        self.get_logger().info("Telemetry & Diagnostics Node Online - Monitoring system status...")

    def handle_reset_fault(self, request, response):
        self.get_logger().info("External fault reset requested via Telemetry Node. Forwarding to hardware driver...")
        
        if not self.reset_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Hardware fault recovery service is not available!")
            response.success = False
            response.message = "Hardware driver service not ready."
            return response
            
        req = ExampleTrigger.Request()
        result = self.reset_client.call(req)
        
        if result.success:
            self.get_logger().info(f"Successfully cleared hardware faults: {result.message}")
            response.success = True
            response.message = result.message
        else:
            self.get_logger().error(f"Failed to clear hardware faults: {result.message}")
            response.success = False
            response.message = result.message
            
        return response

    def report_callback(self, msg: ExtendedStatus):
        """Update local registry with incoming heartbeat and log fault transitions."""
        node_name = msg.node_name
        
        if node_name in self.tracked_nodes:
            prev_state = self.tracked_nodes[node_name]["msg"].state
            if msg.state == ExtendedStatus.STATE_FAULT and prev_state != ExtendedStatus.STATE_FAULT:
                self.get_logger().error(f"Node '{node_name}' has entered FAULT state: {msg.status_message}")
        elif msg.state == ExtendedStatus.STATE_FAULT:
            self.get_logger().error(f"Node '{node_name}' has entered FAULT state: {msg.status_message}")

        # Workaround self-healing activation trigger
        if node_name == 'kinova_hardware_client' and "fault_controller is NOT active" in msg.status_message:
            if not self.activating_fault_controller:
                self.activating_fault_controller = True
                self.get_logger().warn("Detected inactive fault_controller! Initiating automated self-healing activation...")
                self.activate_fault_controller()

        self.tracked_nodes[node_name] = {
            "msg": msg,
            "last_seen": self.get_clock().now()
        }

    def activate_fault_controller(self):
        """Launches a background thread to safely configure and activate fault_controller."""
        threading.Thread(target=self._activate_fault_controller_thread, daemon=True).start()

    def _activate_fault_controller_thread(self):
        try:
            self.get_logger().info("Self-healing: Waiting for /controller_manager services...")
            
            # 1. Configure the controller
            if not self.configure_ctrl_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("Self-healing error: Configure controller service not available!")
                self.activating_fault_controller = False
                return
            
            # Dynamically instantiate request to resolve IDE 'Unresolved attribute reference' warnings
            config_req = self.configure_ctrl_client.srv_type.Request()
            config_req.name = 'fault_controller'
            self.get_logger().info("Self-healing: Configuring fault_controller...")
            config_future = self.configure_ctrl_client.call_async(config_req)
            
            # Use native OS-level thread blocking (threading.Event) instead of polling or CPU-heavy while loops
            config_event = threading.Event()
            config_future.add_done_callback(lambda f: config_event.set())
            config_event.wait()
                
            config_res = config_future.result()
            if config_res and config_res.ok:
                self.get_logger().info("Self-healing: successfully configured fault_controller!")
            else:
                self.get_logger().error("Self-healing warning: Configure controller returned failure (might already be configured). Continuing...")
            
            # 2. Activate the controller
            if not self.switch_ctrl_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("Self-healing error: Switch controller service not available!")
                self.activating_fault_controller = False
                return
                
            # Dynamically instantiate request to resolve IDE 'Unresolved attribute reference' warnings
            switch_req = self.switch_ctrl_client.srv_type.Request()
            switch_req.activate_controllers = ['fault_controller']
            switch_req.deactivate_controllers = []
            switch_req.strictness = SwitchController.Request.STRICT
            
            self.get_logger().info("Self-healing: Activating fault_controller...")
            switch_future = self.switch_ctrl_client.call_async(switch_req)
            
            # Use native OS-level thread blocking (threading.Event) instead of polling or CPU-heavy while loops
            switch_event = threading.Event()
            switch_future.add_done_callback(lambda f: switch_event.set())
            switch_event.wait()
                
            switch_res = switch_future.result()
            if switch_res and switch_res.ok:
                self.get_logger().info("Self-healing: successfully activated fault_controller! System is now self-healed.")
            else:
                self.get_logger().error("Self-healing error: Failed to activate fault_controller.")
                
        except rclpy.exceptions.ROSInterruptException:
            self.get_logger().warn("Self-healing: Interrupted because ROS is shutting down.")
        except Exception as e:
            self.get_logger().error(f"Self-healing error: Unexpected failure during fault_controller activation: {e}")
        finally:
            self.activating_fault_controller = False

    def aggregate_and_publish(self):
        """Computes summary state using the 'Worst-Case Wins' priority rule and broadcasts it."""
        summary = SystemSummary()
        current_time = self.get_clock().now()
        
        has_busy = False
        has_fault = False

        # If zero nodes have checked in yet, flag as system fault (not ready)
        if not self.tracked_nodes:
            summary.summary_state = SystemSummary.SYSTEM_FAULT
            self.pub.publish(summary)
            return

        for node_name, info in self.tracked_nodes.items():
            msg = info["msg"]
            last_seen = info["last_seen"]

            # Calculate the time difference using rclpy Duration
            time_diff = current_time - last_seen

            # Check for stale node crash (Missing heartbeat > 1.5 seconds)
            if time_diff.nanoseconds > self.heartbeat_timeout.nanoseconds:
                has_fault = True
                msg.state = ExtendedStatus.STATE_FAULT
                msg.status_message = f"CRITICAL: Node heartbeat timed out (Stale for {round(time_diff.nanoseconds / 1e9, 2)}s)"
                msg.last_command_valid = False

            if msg.state == ExtendedStatus.STATE_FAULT:
                has_fault = True
            elif msg.state == ExtendedStatus.STATE_BUSY:
                has_busy = True

            summary.individual_states.append(msg)

        # Apply "Worst-Case Wins" Priority Rule Matrix
        if has_fault:
            summary.summary_state = SystemSummary.SYSTEM_FAULT
        elif has_busy:
            summary.summary_state = SystemSummary.SYSTEM_BUSY
        else:
            summary.summary_state = SystemSummary.SYSTEM_READY

        self.pub.publish(summary)

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    executor = MultiThreadedExecutor(num_threads=10) # TODO (pulkit): remove hardcoded thread count
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
