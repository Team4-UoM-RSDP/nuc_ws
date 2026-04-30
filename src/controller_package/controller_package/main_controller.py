import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
from sklearn.cluster import DBSCAN
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.task import Future

from controller_interfaces import TurnExploreOn
from controller_interfaces import TurnExploreOff
from controller_interfaces import ControllerSet
from controller_interfaces import ControllerPositionSet
from controller_interfaces import DetectedObjects
from controller_interfaces import DetectObjectsOn
from controller_interfaces import DetectObjectsOff
from controller_interfaces import ExploringComplete

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
        

        #Nav
        self.service_client_robot_explore_on = self.create_client(
             srv_type=TurnExploreOn,
             srv_name='/turn_explore_on')
        
        self.service_client_robot_explore_off = self.create_client(
             srv_type=TurnExploreOff,
             srv_name='/turn_explore_off')
        

        #Cobot
        self.service_client_controller_set = self.create_client(
            srv_type=ControllerSet,
            srv_name='/controller_set')
        
        self.service_client_controller_position_set = self.create_client(
            srv_type=ControllerPositionSet,
            srv_name='/controller_position_set')
        
        #initialise service server
        #Nav
        self.service_server_exploring_complete = self.create_service(
            srv_type=ExploringComplete,
            srv_name='/exploring_complete',
            callback=self.exploring_complete_service_callback)
        
        #####################################################################
        
        #future variables
        self.explore_on_future=None
        self.explore_off_future=None

        self.controller_set_future=None
        self.controller_position_set_future=None

        self.dectect_objects_on_future=None
        self.dectect_objects_off_future=None


        #Logic variables
        self.explore_requested=False
        self.manipulator_start=False
        self.exploring_complete=False

        #Object detection lists
        self.initial_object_list=[]
        
        
        ########################################
        #Wait for service servers to come online

        #Nav
        while not self.service_client_robot_explore_on.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {self.service_client_robot_explore_on.srv_name} not available, waiting...')
        while not self.service_client_robot_explore_off.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {self.service_client_robot_explore_off.srv_name} not available, waiting...')
        while not self.service_server_exploring_complete.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {self.service_server_exploring_complete.srv_name} not available, waiting...')
            
        #Cobot
        while not self.service_client_controller_set.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {self.service_client_robot_controller_set.srv_name} not available, waiting...')
        while not self.service_client_controller_position_set.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {self.service_client_controller_position_set.srv_name} not available, waiting...')
        

        #Object detection
        while not self.object_detection_on.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {self.object_detection_on.srv_name} not available, waiting...')
        while not self.object_detection_off.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {self.object_detection_off.srv_name} not available, waiting...')

        
        #######################################
    

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
                
                if self.manipulator_start == False:
                    request_manipulator_config_1 = ControllerSet.Request()
                    request_manipulator_config_1.config = 1
                    
                    if self.controller_set_future is not None and not self.controller_set_future.done():
                        self.controller_set_future.cancel()
                        self.get_logger().warn('Config 1 cancelled is controller node still running? ')
                        
                        
                    self.controller_set_future=self.service_client_robot_on.call_async(request_manipulator_config_1)
                    self.controller_set_future.add_done_callback(self.config_1_set)


                #start exploration
                if self.manipulator_start == True:
                    request_explore_on = TurnExploreOn.Request()
                    
                    if self.explore_on_future is not None and not self.explore_on_future.done():
                        self.explore_on_future.cancel()
                        self.get_logger().warn('Explore on cancelled is nav node still running? ')
                        
                        
                    self.explore_on_future = self.service_client_robot_on.call_async(request_explore_on)
                    self.explore_on_future.add_done_callback(self.after_explore_on)
                #start object detection
                if self.explore_requested == True:
                    request_object_detection_on = DetectObjectsOn.Request()
                    
                    if self.dectect_objects_on_future is not None and not self.dectect_objects_on_future.done():
                        self.dectect_objects_on_future.cancel()
                        self.get_logger().warn('Object detection on cancelled is camera node still running? ')
                        
                        
                    self.dectect_objects_on_future=self.service_client_robot_on.call_async(request_object_detection_on)
                    self.dectect_objects_on_future.add_done_callback(self.after_object_detection_on)
            case 2:
                #object detection storing returns 
                self.store_initial_object_list = True
                #Once exploring has complete turn off object detection and generate list of detected clusters 
                #in order of largest to smallest no. of cluster points 
                if self.exploring_complete == True:
                    self.store_initial_object_list = False
                    request_object_detection_off = DetectObjectsOff.Request()
                    
                    if self.dectect_objects_off_future is not None and not self.dectect_objects_off_future.done():
                        self.dectect_objects_off_future.cancel()
                        self.get_logger().warn('Object detection off cancelled is camera node still running? ')
                        
                        
                    self.dectect_objects_off_future=self.service_client_robot_on.call_async(request_object_detection_off)
                    self.dectect_objects_off_future.add_done_callback(self.after_initial_object_detection_off)
            case 3:
                #uses /goal_pose to navigate to grasping reach from the detected object at the top of the found list
                current_position_check=self.initial_object_list.pop(0)
                pass
            case 4:
                #sends manipulator to close scan spin
                #starts object detection to close check for block
                #if object detected 10 times over a set period go to case 5
                #else go back to case 3
                pass
            case 5:
                #use found average position of block
                #manipulator grabs block 
                #once block is grabbed(service response callback )go case 6
                pass
            case 6:
                #return to starting position
                #use object detection for sorting bin
                #if sorting bin detected go case 7
                #else case 8
                pass
            case 7:
                #move to placing distance
                #call manipulator to place 
                #if blocks_returned=3 end
                #else back to case 3
                pass
                
            
            case 8:
                #rotate rover 45 degrees
                
                #if rotation hit count 3 move backwards 30cm
                #back to case 6
                pass
                
                    

                    
    #format the initial list to remove duplicates and any objects detected within a 20cm radius of eachother 
    # are averaged to a single object
    def after_initial_object_detection_off(self,future):
        response = future.result()
        if response.success == True:
            
            # eps=0.2 (20cm), min_samples=1 (Keep everything)
            db = DBSCAN(eps=0.2, min_samples=1).fit(self.initial_object_list)
            labels = db.labels_

            # Create a list of dictionaries containing cluster center and no points in cluster
            results = []
            for label in set(labels):
                mask = (labels == label)
                cluster_points = self.initial_object_list[mask]
                
                results.append({
                    'position': cluster_points.mean(axis=0),
                    'count': len(cluster_points),
                    
                })

            # Sort the entire list by count (highest density first)
            results.sort(key=lambda x: x['count'], reverse=True)
            self.initial_object_list=[]
            for item in results:
                self.initial_object_list.append(item['position'])

            self.initial_object_list = results
            self.current_case = 3
            

    def exploring_complete_service_callback(self,request,response):

        response.success=True
        self.exploring_complete=True
        return response


    def record_detected_object_position(self,msg:DetectedObjects):
        if self.store_initial_object_list==True:
            msg_array=[msg.x,msg.y,msg.z]
            self.initial_object_list.append(msg_array)





    def after_object_detection_on(self,future):
        response=future.result()
        if response.success==True:
            #next case
            self.current_case=2
            #reset logic variables
            self.manipulator_start==False
            self.explore_requested=False



    def config_1_set(self,future):
        response=future.result()
        if response.success==True:
            self.manipulator_start==True
            self.controller_set_future=None


    #Prevents multiple explore on services being sent                
    def after_explore_on(self,future:Future):
        response=future.result()
        if response.success==True:
            self.explore_requested=True
            self.explore_on_future=None
            


           
    
def main(args=None):
    try:
        rclpy.init(args=args)
        omega_operator_controller_node=OmegaOperatorControllerNode()
        rclpy.spin(omega_operator_controller_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
if __name__=='__main__':
    main()
'/rplidar_node/scan/rvis2/initialpose'