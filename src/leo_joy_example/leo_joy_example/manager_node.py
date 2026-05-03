import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import subprocess
import os
import signal

class JoyLauncher(Node):
    def __init__(self):
        super().__init__('joy_launcher_manager')
        self.publisher_leo_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.process = None
        self.button_index = 2 #square
        self.last_button_state = 0
        self.stop_count=0
        self.get_logger().info("Manager Node Started. Waiting for button press...")

    def joy_callback(self, msg):
        current_button_state = msg.buttons[self.button_index]

        
        if current_button_state == 1 and self.last_button_state == 0:
            self.get_logger().info("button pressed has been pressed:")
            if self.process is None:
                self.start_nodes()
            else:
                self.stop_nodes()
        
        self.last_button_state = current_button_state

    def start_nodes(self):
        self.get_logger().info("Launching Rover ros2 Nodes...")
        # Replace 'leo_joy_example' and 'joy.launch.py' with your specific package/file
        self.process = subprocess.Popen(
            ['ros2', 'run', 'demonstration_controller', 'object_detect_and_pick'],
            preexec_fn=os.setsid # Allows killing the whole process group
        )

    def stop_nodes(self):
        if self.process:
            self.get_logger().info("Stopping Nodes (Manual Takeover Enabled)...")
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
            self.process = None
            stop_timer_period: float = 1/100
            self.stop_timer = self.create_timer(stop_timer_period, self.stop_functions_publisher_callback)
            
           

    def stop_functions_publisher_callback(self):
        
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_leo_vel.publish(msg)

        if self.stop_count==100:
            self.stop_timer.cancel()
            self.stop_count=0
        
        self.stop_count+=1


            

def main(args=None):
    try:
        rclpy.init(args=args)
        node = JoyLauncher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
if __name__=='__main__':
    main()   