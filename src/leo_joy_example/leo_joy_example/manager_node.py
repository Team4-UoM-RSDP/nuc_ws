import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
import subprocess
import os
import signal

class JoyLauncher(Node):
    def __init__(self):
        super().__init__('joy_launcher_manager')
        self.publisher_leo_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cobot_start_stop = self.create_publisher(Int8, '/cobot_start_stop', 10)
        
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.process = None
        self.button_index = 0 #square
        self.last_button_state = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.start_count = 0
        self.stop_count = 0
        self.get_logger().info("Manager Node Started. Waiting for button press...")

    def joy_callback(self, msg):

        map=[10,0,1,2,3,11,12,13,14,6]
        for i in map:
            self.button_index = i
            current_button_state = msg.buttons[self.button_index]
            if current_button_state == 1 and self.last_button_state[i] == 0:
                self.get_logger().info(f"button {self.button_index} pressed has been pressed:")
                
                if self.button_index == 10:
                    self.stop_nodes()
                    self.last_button_state[i] = current_button_state
                if self.process is None and self.button_index !=10:
                    self.start_nodes()
                    self.last_button_state[i] = current_button_state
                break
            self.last_button_state[i] = current_button_state
                
            
                
                
        
        
        
        

    def start_nodes(self):
        
        # Replace 'leo_joy_example' and 'joy.launch.py' with your specific package/file
        match self.button_index:
            case 0:
                file = ['ros2', 'run', 'demonstration_controller', 'object_detect_and_pick']
            case 1:
                file = ['ros2', 'run', 'demonstration_controller', 'look_then_pick_just_manipulator']
            case 2:
                file = ['ros2', 'run', 'demonstration_controller', 'move_to_block_pick']
            case 3:
                file = ['ros2', 'run', 'demonstration_controller', 'object_detect_pick_place']
            case 11:
                file = ['ros2', 'run', 'demo_scripts', 'demo_drop']
            case 12:
                file = ['ros2', 'run', 'demo_scripts', 'demo_pick_vision']
            case 13:
                file = ['ros2', 'run', 'demo_scripts', 'demo_pick']
            case 14:
                file = ['ros2', 'run', 'demo_scripts', 'demo_scan']
            case 6:
                file = ['ros2', 'run', 'demonstration_controller', 'demo_takeover']

            
        self.get_logger().warn(f"Launching Rover ros2 Node: {file}")
        self.process = subprocess.Popen(
            file,
            preexec_fn=os.setsid # Allows killing the whole process group
        )
        start_timer_period: float = 1/100
        self.start_timer = self.create_timer(start_timer_period, self.start_functions_publisher_callback)
        

    def stop_nodes(self):
            if self.process  == None:
                pass
            else:
                self.get_logger().info("Stopping Nodes (Manual Takeover Enabled)...")
                os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
                
                stop_timer_period: float = 1/100
                self.stop_timer = self.create_timer(stop_timer_period, self.stop_functions_publisher_callback)
                
            
           
    def start_functions_publisher_callback(self):
        
        msg = Int8()
        msg.data = 0
        self.cobot_start_stop.publish(msg)

        if self.start_count==100:
            self.start_timer.cancel()
            self.start_count=0
        
        self.start_count+=1


    def stop_functions_publisher_callback(self):

        msg = Int8()
        msg.data = 1
        self.cobot_start_stop.publish(msg)
        
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_leo_vel.publish(msg)

        if self.stop_count==100:
            self.stop_timer.cancel()
            self.stop_count=0
            self.process = None
        
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