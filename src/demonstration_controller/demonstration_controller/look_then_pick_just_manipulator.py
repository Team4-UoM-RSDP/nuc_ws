import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
from sklearn.cluster import DBSCAN
import time

from controller_interfaces import ControllerSet
from controller_interfaces import ControllerPositionSet
from controller_interfaces import DetectedObjects
from controller_interfaces import DetectObjectsOn
from controller_interfaces import DetectObjectsOff


from geometry_msgs.msg import Twist



class ControllerNode(Node):
    
    def __init__(self):
        super().__init__('leo_controller')
        self.current_case=0

        self.starting_pos=None

        """ self.task_space_pose_subscriber = self.create_subscription(
            msg_type = TaskSpacePose,
            topic='/omega_operator/task_space_pose',
            qos_profile=1,
            callback=self.task_space_pose_subscriber_callback)"""
        

        #################################
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.controller_set_future=None
        msg = Twist()
        msg.angular.z = 10.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: x vel : {msg.linear.x}' )

        #############
    

        #TF pose listener
        self.transform_listener_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.transform_listener_buffer, self)
        self.parent_name="world"
        self.child_name="leo1/base_link"

        ###################################################
        #Topic and Service initialisation

        #initialise topic subscribers
        self.object_detection_subscriber = self.create_subscription(
            msg_type=DetectedObjects,
            topic='/detected_objects',
            callback=self.record_detected_object_position,
            qos_profile=1)
        
        
        
        
        #Initialise the service clients

        #Object detection
        self.object_detection_on = self.create_client(
             srv_type=DetectObjectsOn,
             srv_name='/turn_object_detection_on')
        
        self.object_detection_off = self.create_client(
             srv_type=DetectObjectsOff,
             srv_name='/turn_object_detection_off')
        
        

        #Cobot
        self.service_client_controller_set = self.create_client(
            srv_type=ControllerSet,
            srv_name='/controller_set')
        
        self.service_client_controller_position_set = self.create_client(
            srv_type=ControllerPositionSet,
            srv_name='/controller_position_set')
        
        
        
        #####################################################################
        
        #future variable

        self.controller_set_future=None
        self.controller_position_set_future=None

        self.dectect_objects_on_future=None
        self.dectect_objects_off_future=None


        #Logic variables
        
        self.manipulator_start=False
        self.store_initial_object_list = False

        self.case_2_timer_create=False
        self.config_looper=0
        
        #Object detection lists
        self.initial_object_list=[]
        
        
        
        ########################################
        #Wait for service servers to come online

        
        #Cobot
        while not self.service_client_controller_set.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {self.service_client_controller_set.srv_name} not available, waiting...')
        while not self.service_client_controller_position_set.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {self.service_client_controller_position_set.srv_name} not available, waiting...')
        

        #Object detection
        while not self.object_detection_on.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {self.object_detection_on.srv_name} not available, waiting...')
        while not self.object_detection_off.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {self.object_detection_off.srv_name} not available, waiting...')

        
        #######################################
        timer_period: float = 1/120
        self.timer = self.create_timer(timer_period, self.main_loop_callback)
    

    def main_loop_callback(self):
        match self.current_case:
            case 0:
                #get the starting position before moving onto anything else

                
                try:
                        tfs =   self.transform_listener_buffer.lookup_transform(
                                self.parent_name,
                                self.child_name,
                                rclpy.time.Time())
                        self.starting_pos=tfs
                        self.get_logger().info(f"Starting Pose: {tfs}")
                       

                except tf2_ros.TransformException as e:

                        self.get_logger().error(
                            f'Could not get transform from `{self.parent_name}` to `{self.child_name}`: {e}')
                self.current_case=1


            case 1:
                
                #manipulator switches between far scan 0 and 1
                
                if self.controller_set_future == None:
                    self.controller_set_config(2,self.config_1_set)
                    


            
            case 2:
                #object detection storing returns 
                if self.controller_set_future == None:
                    self.controller_set_config(3,self.config_2_set)
                    
                    
            case 3:
                #uses grasp block at given position
                if self.controller_set_future == None:
                    self.controller_set_config(5,self.config_3_set)
                    
                
                
            case 4:
                self.get_logger().info("Sequence complete.")
                self.timer.cancel()
                


    #####################################################################################################################################
    #keep these

    def controller_set_config(self,config:int,callback):
        request_manipulator_config = ControllerSet.Request()
        request_manipulator_config.config = config 
                    
        if self.controller_set_future is not None and not self.controller_set_future.done():
            self.controller_set_future.cancel()
            self.get_logger().warn(f'Config {config} cancelled is controller node still running? ')
                        
                        
        self.controller_set_future=self.service_client_controller_set.call_async(request_manipulator_config)
        self.controller_set_future.add_done_callback(callback)
    
    def config_1_set(self,future):
        response=future.result()
        if response.success==True:
            timer_period: float = 1/100
            self.timer_config_1 = self.create_timer(timer_period, self.timer_config_1_callback)#
        else:
            self.controller_set_future=None
            self.get_logger().info("config 1 set failed")


    
    def timer_config_1_callback(self):
        if self.config_looper<3:
            self.config_looper+=1
            self.current_case=1
            self.controller_set_future=None
            self.timer_config_1.cancel()
        
        else:
            time.sleep(2)
            self.config_looper=0
            self.current_case=2
            self.controller_set_future=None
            self.timer_config_1.cancel()
    

    def config_2_set(self,future):
        response=future.result()
        if response.success==True:
            timer_period: float = 15
            self.timer_config_2 = self.create_timer(timer_period, self.timer_config_2_callback)#
        else:
            self.controller_set_future=None
            self.get_logger().info("config 2 set failed")


    
    def timer_config_2_callback(self):
        self.current_case=3
        self.controller_set_future=None
        self.timer_config_2.cancel()

    def config_3_set(self,future):
        response=future.result()
        if response.success==True:
            self.current_case=4
            self.controller_set_future=None
            
        else:
            self.controller_set_future=None
            self.get_logger().info("config 3 set failed")


    
   
            

    
    
            



    #####################################################################################################################################


           
    
def main(args=None):
    try:
        rclpy.init(args=args)
        omega_operator_controller_node=ControllerNode()
        rclpy.spin(omega_operator_controller_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
if __name__=='__main__':
    main()
