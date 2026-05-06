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


class ControllerNode(Node):
    
    def __init__(self):
        super().__init__('leo_controller')
        self.current_case=0

        

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

        self.leo_velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        

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
        
        
        self.store_initial_object_list = False

        
       
        
        self.number_of_detection_scans=2

        self.config_count=0
        self.back_n_spin_count=0
        
      



        
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
        timer_period: float = 1/20
        self.timer = self.create_timer(timer_period, self.main_loop_callback)
    

    def main_loop_callback(self):
        match self.current_case:
            case 0:
                self.current_case=4
                #get the starting position before moving onto anything else

                
                try:
                        tfs =   self.transform_listener_buffer.lookup_transform(
                                self.parent_name,
                                self.child_name,
                                rclpy.time.Time())
                        
                        self.get_logger().info(f"Starting Pose: {tfs}")
                       

                except tf2_ros.TransformException as e:

                        self.get_logger().error(
                            f'Could not get transform from `{self.parent_name}` to `{self.child_name}`: {e}')
                

            #case 1-3 look near stationary, detect,pick
            case 1:
                self.current_case=99
                #manipulator switches between far scan 0 and 1
                self.controller_set_config(4,self.config_4_set)
                
                    

                #start object detection
            case 11:
                    self.case=99
                    request_object_detection_on = DetectObjectsOn.Request()
                    
                    if self.dectect_objects_on_future is not None and not self.dectect_objects_on_future.done():
                        self.dectect_objects_on_future.cancel()
                        self.get_logger().warn('Object detection on cancelled is camera node still running? ')
                        
                        
                    self.dectect_objects_on_future=self.object_detection_on.call_async(request_object_detection_on)
                    self.dectect_objects_on_future.add_done_callback(self.after_object_detection_on)
                    
            case 2:
                #object detection storing returns 
                    self.case=99
                    self.store_initial_object_list = True
                    #after 20 seconds turn off object detection and generate list of detected clusters 
                    #in order of largest to smallest no. of cluster points 
                    
                    timer_period: float = 20
                    self.object_detection_timer = self.create_timer(timer_period, self.object_detection_timer_callback)
                    
                    
                    
            case 3:
                #uses grasp block at given position
                
                    try:
                        self.current_block_position=self.initial_object_list.pop(0)
                        self.current_case=33
                    except:
                        self.current_case=1
                        self.initial_object_list=[]
                    
                        
            case 33:
                    self.case=99
                    request_object_pick = ControllerPositionSet.Request()
                    request_object_pick.x=self.current_block_position[0]
                    request_object_pick.y=self.current_block_position[1]
                    request_object_pick.z=self.current_block_position[2]
                        
                    if self.controller_position_set_future is not None and not self.controller_position_set_future.done():
                            self.controller_position_set_future.cancel()
                            self.get_logger().warn('Controller position set on cancelled is controller node still running? ')
                            
                            
                    self.controller_position_set_future=self.service_client_controller_position_set.call_async(request_object_pick)
                    self.controller_position_set_future.add_done_callback(self.after_controller_position_set)
                    

                
                
            case 4:
                

                    self.current_case=99
                    request_object_detection_on = DetectObjectsOn.Request()
                    
                    
                    
                    if self.dectect_objects_on_future is not None and not self.dectect_objects_on_future.done():
                            self.dectect_objects_on_future.cancel()
                            self.get_logger().warn('Object detection on cancelled is camera node still running? ')
                            
                            
                    self.dectect_objects_on_future=self.object_detection_on.call_async(request_object_detection_on)
                    self.dectect_objects_on_future.add_done_callback(self.after_object_detection_scanning)
                    
            case 44:

                    self.current_case=99
                    self.controller_set_config(1,self.config_1_set)
                    
                
            
            case 5:
                    self.current_case=99
                    request_object_detection_off = DetectObjectsOff.Request()
                            
                    if self.dectect_objects_off_future is not None and not self.dectect_objects_off_future.done():
                        self.dectect_objects_off_future.cancel()
                        self.get_logger().warn('Object detection off cancelled is camera node still running? ')
                                    
                                    
                    self.dectect_objects_off_future=self.object_detection_off.call_async(request_object_detection_off)
                    self.dectect_objects_off_future.add_done_callback(self.after_object_detection_scanning_off)
                    
                    
            case 6:
                
                    
                    try:
                        self.current_block_target=self.initial_object_list.pop(0)
                        self.back_n_spin_count = 0
                        self.current_case=66
                    except:
                        
                       
                        
                        self.initial_object_list=[]
                        self.current_case=4
                        return
            case 66:
                    self.current_case=99
                    self.current_block_target[0]=self.current_block_target[0]-0.0305
                    self.current_block_target[1]=self.current_block_target[1]#adjust this once you know the frame of the manipulators y direction in respect to the rovers
                    angle=np.arctan2(self.current_block_target[1],self.current_block_target[0])
                    self.angle_in_4_seconds=angle/4
                    distance=np.sqrt(self.current_block_target[0]**2+self.current_block_target[1]**2)-0.218
                    self.distance_in_4_seconds=distance/4

                    
                    timer_period_back_n_spin: float = 1/50
                    
                        
                    self.timer_for_back_n_spin = self.create_timer(timer_period_back_n_spin, self.back_n_spin_callback)
                    
            
            case 98:
                self.get_logger().info('Operation complete...')
                self.timer.cancel()

            case 99:
                pass
                    

                


    #####################################################################################################################################
    #keep these

    def back_n_spin_callback(self):
        
        if self.back_n_spin_count <= 200:
            spin=Twist()
            spin.angular.z = self.angle_in_4_seconds
            self.leo_velocity_publisher.publish(spin)

        elif self.back_n_spin_count <= 400:
            going_forward=Twist()
            going_forward.angular.z = 0
            going_forward.linear.x = self.distance_in_4_seconds
            self.leo_velocity_publisher.publish(going_forward)
        else:
            going_forward=Twist()
            going_forward.angular.z = 0
            going_forward.linear.x = 0
            self.leo_velocity_publisher.publish(going_forward)
            if self.distance_in_4_seconds*4>0.2:
                self.current_case=4
               
                self.back_n_spin_count = 0
                self.timer_for_back_n_spin.cancel()
                
                self.initial_object_list=[]
            else:
                self.current_case=1
                
                self.back_n_spin_count = 0
                self.timer_for_back_n_spin.cancel()
                
                self.initial_object_list=[]


        

        self.back_n_spin_count+=1

    def after_object_detection_scanning_off(self,future):
        response = future.result()
        if response.success == True:
            results=self.clustering(0.1,self.initial_object_list)
            if results==None:
                self.current_case=4
                self.initial_object_list=[]
                self.number_of_detection_scans+=1
                self.get_logger().warn('clustering failed')
            else:

                self.initial_object_list=[]
                self.number_of_detection_scans=2
                for item in results:
                    self.initial_object_list.append(item['position'])

                
                self.current_case = 6
                
                self.dectect_objects_off_future = None
                
        if response.success==False:
            self.get_logger().info('object detection off failed...\nTrying again')
            self.dectect_objects_off_future = None
            self.current_case=5

    def after_object_detection_scanning(self,future):
        response=future.result()
        if response.success==True:
            self.get_logger().info('object detection on')
            self.current_case=44
            self.dectect_objects_on_future = None
        if response.success==False:
            self.dectect_objects_on_future=None
            self.get_logger().warn('object detection on failed....\nTrying again')
            self.current_case=4


    def config_1_set(self,future):
        response=future.result()
        if response.success==True:

            
            self.config_count+=1
            if self.config_count<self.number_of_detection_scans:
                self.controller_set_future=None
                self.controller_set_config(2,self.config_1_set)
            else:
                
                self.controller_set_future=None
                self.current_case = 5
                
                self.config_count=0
                
        elif response.success==False:
            self.controller_set_future=None
            self.get_logger().warn('Config 2 failed....\nTrying again')
            self.current_case=44

    def controller_set_config(self,config:int,callback):
        request_manipulator_config = ControllerSet.Request()
        request_manipulator_config.config = config 
                    
        if self.controller_set_future is not None and not self.controller_set_future.done():
            self.controller_set_future.cancel()
            self.get_logger().warn(f'Config {config} cancelled is controller node still running? ')
                        
                        
        self.controller_set_future=self.service_client_controller_set.call_async(request_manipulator_config)
        self.controller_set_future.add_done_callback(callback)
    
    def config_4_set(self,future):
        response=future.result()
        if response.success==True:
            
            self.controller_set_future=None
            self.current_case = 11
        if response.success==False:
            self.current_case=1
            self.controller_set_future=None
            

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
            self.dectect_objects_on_future = None
        if response.success==False:
            #next case
            self.current_case=11
            self.dectect_objects_on_future = None
            
            

    def object_detection_timer_callback(self):
        self.store_initial_object_list = False
        if self.dectect_objects_off_future == None:
        
            request_object_detection_off = DetectObjectsOff.Request()
                        
            if self.dectect_objects_off_future is not None and not self.dectect_objects_off_future.done():
                self.dectect_objects_off_future.cancel()
                self.get_logger().warn('Object detection off cancelled is camera node still running? ')
                            
                            
            self.dectect_objects_off_future=self.object_detection_off.call_async(request_object_detection_off)
            self.dectect_objects_off_future.add_done_callback(self.after_object_detection_off)

    def after_object_detection_off(self,future):
        response = future.result()
        if response.success == True:
            self.object_detection_timer.cancel()
            results=self.clustering(0.1,self.initial_object_list)
            if results==None:
                self.current_case=1
                self.get_logger().warn('clustering failed ')
            else:


                self.initial_object_list=[]
                for item in results:
                    self.initial_object_list.append(item['position'])

                
                self.current_case = 3
                

                self.dectect_objects_off_future = None
        elif response.success==False:
            self.dectect_objects_off_future == None

    
    def after_controller_position_set(self,future):
        response=future.result()
        if response.success==True:
            self.controller_set_config(6,self.config_6)
            #next case
            
            #reset logic variables
    
            
            self.controller_position_set_future=None
        if response.success==False:
            #next case
            self.current_case=33
            #reset logic variables
            
            self.controller_position_set_future=None
    def config_6(self,future):
            response=future.result()
            if response.success==True:
                self.current_case=98
                self.controller_position_set_future=None
            else:
                self.controller_set_config(6,self.config_6)
                self.controller_position_set_future=None

    def clustering(self,eps,list,min_samples=1):
        list=np.array(list)
        try:
            db = DBSCAN(eps=eps, min_samples=min_samples).fit(list)
            labels = db.labels_

                # Create a list of dictionaries containing cluster center and no points in cluster
            results = []
            for label in set(labels):
                mask = (labels == label)
                cluster_points = list[mask]
                
                results.append({
                        'position': cluster_points.mean(axis=0),
                        'count': len(cluster_points),
                        
                    })

            # Sort the entire list by count (highest density first)
            results.sort(key=lambda x: x['count'], reverse=True)
        except:
            results=None
        return results

    
            



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
