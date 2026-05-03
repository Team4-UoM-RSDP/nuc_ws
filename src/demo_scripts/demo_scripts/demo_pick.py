import rclpy
from rclpy.task import Future
from rclpy.node import Node
from rclpy.logging import get_logger

from controller_interfaces.srv import ControllerSet

_logger = get_logger(__name__)


class DemoPick(Node):
    """A ROS2 ControllerSet service client that requests pick block."""

    def __init__(self):
        super().__init__("demo_pick")

        self.service_client = self.create_client(
            srv_type=ControllerSet, srv_name="/controller_set"
        )

        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f"Service {self.service_client.srv_name} not available, waiting..."
            )

        self.future: Future = None

        timer_period: float = 1.0
        self.timer = self.create_timer(
            timer_period_sec=timer_period, callback=self.timer_callback
        )

        self.request_sent = False

    def timer_callback(self):
        """Method that is periodically called by the timer."""
        # Only send request once
        if self.request_sent:
            return

        request = ControllerSet.Request()
        request.config = 5  # Case 5: pick block (no vision)

        if self.future is not None and not self.future.done():
            self.future.cancel()
            self.get_logger().warn(
                "Service Future canceled. The Node took too long to process the service call. "
                "Is the Service Server still alive?"
            )

        self.future = self.service_client.call_async(request)
        self.future.add_done_callback(self.process_response)
        self.request_sent = True

    def process_response(self, future: Future):
        """Callback for the future, that will be called when it is done."""
        response = future.result()
        if response is not None:
            self.get_logger().info(f"Request completed. Success: {response.success}")
        else:
            self.get_logger().info("Service response was None.")


def main(args=None) -> None:
    """Entry point for the DemoPick service client."""
    try:
        rclpy.init(args=args)
        demo_pick = DemoPick()
        rclpy.spin(demo_pick)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        _logger.error(f"Error: {e}")


if __name__ == "__main__":
    main()
