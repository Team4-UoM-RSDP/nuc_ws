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
from geometry_msgs import PoseStamped

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
        
        #topic publisher
        self.move_to_position_nav = self.create_publisher(
            msg_type=PoseStamped,
            topic='/goal_pose',
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
        self.object_detect_close_scan=False
        self.config_2=False
        
        #Object detection lists
        self.initial_object_list=[]
        self.current_object_position_check=[]
        self.detected_object=[]
        
        
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
                current_position_target=self.initial_object_list.pop(0)
                pose_msg = PoseStamped()
                pose_msg.header.frame_id = 'map'
                pose_msg.header.frame_id = self.get_clock().now().to_msg()

                pose_msg.pose.position.x=current_position_target[0]
                pose_msg.pose.position.y=current_position_target[1]
                pose_msg.pose.position.z=current_position_target[2]

                pose_msg.pose.orientation.x=0.0
                pose_msg.pose.orientation.y=0.0
                pose_msg.pose.orientation.z=0.0
                pose_msg.pose.orientation.w=0.0

                self.move_to_position_nav.publish(pose_msg)
                self.current_case=4
                
            case 4:
                #sends manipulator to close scan spin
                #starts object detection to close check for block
                #if object detected 10 times over a set period go to case 5
                #else go back to case 3
                request_manipulator_config_2 = ControllerSet.Request()
                request_manipulator_config_2.config = 2

                if self.controller_set_future is not None and not self.controller_set_future.done():
                    self.controller_set_future.cancel()
                    self.get_logger().warn('Config 2 cancelled is controller node still running?')

                self.controller_set_future = self.service_client_controller_set.call_async(request_manipulator_config_2)
                self.controller_set_future.add_done_callback(self.after_config_2_set)

                if self.config_2==True:
                     
                    request_object_detection_on = DetectObjectsOn.Request()
                    
                    if self.dectect_objects_on_future is not None and not self.dectect_objects_on_future.done():
                        self.dectect_objects_on_future.cancel()
                        self.get_logger().warn('Object detection on cancelled is camera node still running? ')
                        
                        
                    self.dectect_objects_on_future=self.service_client_robot_on.call_async(request_object_detection_on)
                    self.dectect_objects_on_future.add_done_callback(self.after_object_object_detect_close_scan)

                
            case 5:
                #use found average position of block
                #manipulator grabs block 
                #once block is grabbed(service response callback )
                # #return to starting position then go case 6
                
                request_object_pick = ControllerPositionSet.request()
                if self.dectect_objects_on_future is not None and not self.dectect_objects_on_future.done():
                    self.dectect_objects_on_future.cancel()
                    self.get_logger().warn('Object detection on cancelled is camera node still running?')
                request_object_pick.x=self.detected_object[0]
                request_object_pick.x=self.detected_object[1]
                request_object_pick.x=self.detected_object[2]

                self.self.controller_position_set_future = self.self.service_client_controller_position_set.call_async(request_object_pick)
                self.self.controller_position_set_future.add_done_callback(self.object_picked)
            
                        
                
            case 6:
                
                #use object detection for sorting bin
                #if sorting bin detected go case 7
                #else case 8
                request_object_detection_on = DetectObjectsOn.Request()

                if self.dectect_objects_on_future is not None and not self.dectect_objects_on_future.done():
                    self.dectect_objects_on_future.cancel()
                    self.get_logger().warn('Object detection on cancelled is camera node still running?')

                self.dectect_objects_on_future = self.object_detection_on.call_async(request_object_detection_on)
                self.dectect_objects_on_future.add_done_callback(self.after_bin_detection_on)
                
            case 7:
                #move to placing distance
                #call manipulator to place 
                #if blocks_returned=3 end
                #else back to case 3
                request_place = ControllerPositionSet.Request()
                request_place.x = float(self.bin_position[0])
                request_place.y = float(self.bin_position[1])
                request_place.z = float(self.bin_position[2])

                if self.controller_position_set_future is not None and not self.controller_position_set_future.done():
                    self.controller_position_set_future.cancel()
                    self.get_logger().warn('Place cancelled is controller node still running?')

                self.controller_position_set_future = self.service_client_controller_position_set.call_async(request_place)
                self.controller_position_set_future.add_done_callback(self.after_place)
                
                
            
            case 8:
                #rotate rover 45 degrees
                
                #if rotation hit count 3 move backwards 30cm
                #back to case 6
             
                request_rotate = ControllerSet.Request()
                request_rotate.config = 3  # 45 degree rotation config

                if self.controller_set_future is not None and not self.controller_set_future.done():
                    self.controller_set_future.cancel()
                    self.get_logger().warn('Rotate cancelled is controller node still running?')

                self.controller_set_future = self.service_client_controller_set.call_async(request_rotate)
                self.controller_set_future.add_done_callback(self.after_rotate)

    def object_picked(self,future):
                response = future.result()
                if response.success == True:
                    pose_msg = PoseStamped()
                    pose_msg.header.frame_id = 'map'
                    pose_msg.header.frame_id = self.get_clock().now().to_msg()

                    pose_msg.pose.position.x=self.starting_pose.position.x
                    pose_msg.pose.position.y=self.starting_pose.position.y
                    pose_msg.pose.position.z=self.starting_pose.position.z

                    pose_msg.pose.orientation.x=self.starting_pose.orientation.x
                    pose_msg.pose.orientation.y=self.starting_pose.orientation.y
                    pose_msg.pose.orientation.z=self.starting_pose.orientation.z
                    pose_msg.pose.orientation.w=self.starting_pose.orientation.w

                    self.move_to_position_nav.publish(pose_msg)
                    self.current_case=6
            

    def after_config_2_set(self, future):
        response = future.result()
        if response.success == True:
            self.config_2=True
    def after_grab(self, future):
        response = future.result()
        if response.success == True:
            self.controller_position_set_future = None
            self.blocks_returned += 1
            self.current_case = 6

    def after_bin_detection_on(self, future):
        response = future.result()
        if response.success == True:
            self.dectect_objects_on_future = None
            if self.bin_position is not None:
                self.current_case = 7
            else:
                self.current_case = 8

    def after_place(self, future):
        response = future.result()
        if response.success == True:
            self.controller_position_set_future = None
            if self.blocks_returned >= 3:
                self.get_logger().info('All blocks returned, task complete.')
            else:
                self.current_case = 3

    def after_rotate(self, future):
        response = future.result()
        if response.success == True:
            self.controller_set_future = None
            self.rotation_count += 1
            if self.rotation_count >= 3:
                self.rotation_count = 0
                request_reverse = ControllerSet.Request()
                request_reverse.config = 4  # reverse 30cm config
                self.controller_set_future = self.service_client_controller_set.call_async(request_reverse)
                self.controller_set_future.add_done_callback(self.after_reverse)
            else:
                self.current_case = 6

    def after_reverse(self, future):
        response = future.result()
        if response.success == True:
            self.controller_set_future = None
            self.current_case = 6               

                    
    #format the initial list to remove duplicates and any objects detected within a 20cm radius of eachother 
    # are averaged to a single object
    def after_object_object_detect_close_scan(self,future):
        response = future.result()
        if response.success == True:
            self.object_detect_close_scan=True
            timer_period = 10
            self.timer_for_object_check = self.create_timer(timer_period, self.check_if_object_detected)
    
    def check_if_object_detected(self):
        
        results=self.clustering(0.05,self.current_object_position_check,min_samples=10)
        try:
            for item in results:
                self.current_object_position_check.append(item['position'])
            self.detected_object=self.current_object_position_check.pop(0)
            self.current_case=5
            

            self.timer_for_object_check.cancel()
        except:
            self.current_case=3

            self.timer_for_object_check.cancel()
        request_object_detection_off = DetectObjectsOff.Request()
                    
        if self.dectect_objects_off_future is not None and not self.dectect_objects_off_future.done():
                        self.dectect_objects_off_future.cancel()
                        self.get_logger().warn('Object detection off cancelled is camera node still running? ')
                        
                        
        self.dectect_objects_off_future=self.service_client_robot_on.call_async(request_object_detection_off)
        self.dectect_objects_off_future.add_done_callback(self.object_dection_off)
            
    def object_detection_off(self,future):
        response=future.result()
        if response.success==True:
            pass
        else:
            request_object_detection_off = DetectObjectsOff.Request()
            self.dectect_objects_off_future=self.service_client_robot_on.call_async(request_object_detection_off)
            self.dectect_objects_off_future.add_done_callback(self.object_dection_off)
    def clustering(self,eps,list,min_samples=1):
        db = DBSCAN(eps=eps, min_samples=min_samples).fit(list)
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
        return results




    
    def after_initial_object_detection_off(self,future):
        response = future.result()
        if response.success == True:
            results=self.clustering(0.2,self.initial_object_list)

            self.initial_object_list=[]
            for item in results:
                self.initial_object_list.append(item['position'])

            
            self.current_case = 3
            

    def exploring_complete_service_callback(self,request,response):

        response.success=True
        self.exploring_complete=True
        return response


    def record_detected_object_position(self,msg:DetectedObjects):
        if self.store_initial_object_list==True:
            msg_array=[msg.x,msg.y,msg.z]
            self.initial_object_list.append(msg_array)

        if self.object_detect_close_scan==True:
            msg_array=[msg.x,msg.y,msg.z]
            self.current_object_position_check.append(msg_array)


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
        omega_operator_controller_node=ControllerNode()
        rclpy.spin(omega_operator_controller_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
if __name__=='__main__':
    main()
