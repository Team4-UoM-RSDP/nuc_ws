#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from controller_interfaces.srv import ControllerSet
from controller_interfaces.srv import ControllerPositionSet
from controller_interfaces.msg import DetectedObjects
from controller_interfaces.srv import DetectObjectsOn
from controller_interfaces.srv import DetectObjectsOff
import tf2_ros



class DemoNode(Node):
    """A ROS2 Node that publishes an amazing quote."""

    def __init__(self):
        super().__init__('demo_node')
        self.transform_listener_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.transform_listener_buffer, self)
        self.parent_name="map_frame"
        self.child_name="robot_frame"

        timer_period: float = 1
        self.timer = self.create_timer(timer_period, self.timer_spin)
    


    def timer_spin(self):
        try:
                        tfs =   self.transform_listener_buffer.lookup_transform(
                                self.parent_name,
                                self.child_name,
                                rclpy.time.Time())
                        
                        self.get_logger().info(f"Starting Pose: {tfs}")
                       

        except tf2_ros.TransformException as e:

                        self.get_logger().error(
                            f'Could not get transform from `{self.parent_name}` to `{self.child_name}`: {e}')
        

def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        detect_and_pick_node = DemoNode()

        rclpy.spin(detect_and_pick_node)
    except KeyboardInterrupt:
        pass
        
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
