import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
from sklearn.cluster import DBSCAN

from controller_interfaces import ControllerSet
from controller_interfaces import ControllerPositionSet
from controller_interfaces import DetectedObjects
from controller_interfaces import DetectObjectsOn
from controller_interfaces import DetectObjectsOff

from geometry_msgs.msg import Twist
import random 


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
    

        

        ###################################################
        #Topic and Service initialisation

        #initialise topic subscribers
        self.object_detection_subscriber = self.create_subscription(
            msg_type=DetectedObjects,
            topic='/detected_objects',
            callback=self.record_detected_object_position,
            qos_profile=1)
        
        self.leo_velocity_publisher = self.create_publisher(
            msg_type = Twist,
            topic = '/cmd_vel',
            qos_profile = 1,

        )
        
        
        
        self.object_publish=False
        

        #Object detection
        self.object_detection_on = self.create_service(
             srv_type=DetectObjectsOn,
             srv_name='/turn_object_detection_on',
             callback=self.object_on)

        
        self.object_detection_off = self.create_client(
             srv_type=DetectObjectsOff,
             srv_name='/turn_object_detection_off',
             callback=self.object_off)
        
        self.position_pub = self.create_publisher(
            msg_type=DetectedObjects,
            topic="/detected_objects",
            qos_profile=1,
        )

        self.position_pub = self.create_publisher(
            msg_type=DetectedObjects,
            topic="/detected_objects",
            qos_profile=1,
        )
        
        



        

        #Cobot
        self.service_service_controller_set = self.create_service(
            srv_type=ControllerSet,
            srv_name='/controller_set',
            callback=self.set_callback)
        
        self.service_service_controller_position_set = self.create_service(
            srv_type=ControllerPositionSet,
            srv_name='/controller_position_set',
            callback=self.pos_set_callback)
        
        
        
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
        
        timer_period: float = 1/20
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def set_callback(self,request,response):
            self.object_publish=True
            response.success=True
            return response
    
    def set_callback(self,request,response):
            self.object_publish=True
            response.success=True
            return response
    
    def object_off(self,request,response):
            self.object_publish=False
            response.success=True
            return response
    def timer_callback(self):
        if self.object_publish==True:
            x=random.random()*5
            y=random.random()*5
            z=random.random()*5
            msg=DetectedObjects()
            msg.x=x
            msg.y=y
            msg.z=z
            self.position_pub.publish(msg)

             
        else:
             pass
             
            
    
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
