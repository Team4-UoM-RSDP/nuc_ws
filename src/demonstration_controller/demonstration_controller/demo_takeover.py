#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from controller_interfaces import ControllerSet



class DemoNode(Node):
    """A ROS2 Node that publishes an amazing quote."""

    def __init__(self):
        super().__init__('demo node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.controller_set_future=None
        
        self.service_client_controller_set = self.create_client(
            srv_type=ControllerSet,
            srv_name='/controller_set')
        

        while not self.service_client_controller_set.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {self.service_client_controller_set.srv_name} not available, waiting...')
        
        self.controller_set_config(1,self.config_1_set)

        timer_period: float = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i=0

    def config_1_set(self,future):
        response=future.result()
        if response.success==True:
            
            self.controller_set_future=None
        
        

    def timer_callback(self):
        """Method that is periodically called by the timer."""
        msg = Twist()
        msg.angular.z = 0.5
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: x vel : {msg.linear.x}' )
        self.i += 1

    def controller_set_config(self,config:int,callback):
        request_manipulator_config = ControllerSet.Request()
        request_manipulator_config.config = config 
                    
        if self.controller_set_future is not None and not self.controller_set_future.done():
            self.controller_set_future.cancel()
            self.get_logger().warn(f'Config {config} cancelled is controller node still running? ')
                        
                        
        self.controller_set_future=self.service_client_controller_set.call_async(request_manipulator_config)
        self.controller_set_future.add_done_callback(callback)
        

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
