import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

from controller_interfaces import TurnExploreOn
from controller_interfaces import TurnExploreOff


class ControllerNode(Node):
    def __init__(self):
        super().__init__("leo_controller")
        self.current_case = 0

        self.starting_pos = None

        """ self.task_space_pose_subscriber = self.create_subscription(
            msg_type = TaskSpacePose,
            topic='/omega_operator/task_space_pose',
            qos_profile=1,
            callback=self.task_space_pose_subscriber_callback)"""

        # TF pose listener
        self.transform_listener_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.transform_listener_buffer, self
        )
        self.parent_name = "world"
        self.child_name = "leo1/base_link"
        # initiates the turn on service linking callback

        self.service_client_robot_explore_on = self.create_client(
            srv_type=TurnExploreOn, srv_name="/turn_exploration_on"
        )

        self.service_client_robot_explore_on = self.create_client(
            srv_type=TurnExploreOff, srv_name="/turn_exploration_off"
        )

        timer_period: float = 1
        self.timer = self.create_timer(
            timer_period_sec=timer_period, callback=self.main_loop_callback
        )

    def main_loop_callback(self):
        match self.current_case:
            case 0:
                # get the starting position before moving onto anything else
                loop_till_found = True
                while loop_till_found:
                    try:
                        tfs = self.transform_listener_buffer.lookup_transform(
                            self.parent_name, self.child_name, rclpy.time.Time()
                        )
                        self.starting_pos = tfs
                        self.get_logger().info(f"Starting Pose: {tfs}")
                        loop_till_found = False

                    except tf2_ros.TransformException as e:
                        self.get_logger().error(
                            f"Could not get transform from `{self.parent_name}` to `{self.child_name}`: {e}"
                        )
                self.current_case = 1

            case 1:
                pass
                # start exploration
                # manipulator switches between far scan 0 and 1
                # object detection returns

    def which_move_control(self):
        self.time_for_rotation = 45
        self.time_for_linear = 45  # must be at least distance*10
        if self.is_robot_on == True:
            if self.time_increment < (self.time_for_rotation * 10):
                self.rotation_control()
            elif self.time_increment < (
                (self.time_for_linear + self.time_for_rotation) * 10
            ):
                self.linear_control()
            else:
                self.robot_turn_off()
                self.get_logger().info("Robot movement complete switching off")
                self.is_robot_on = False

    def rotation_control(self):

        angle: float = -69
        angle_rad = np.deg2rad(angle)
        update_frequency = 0.1

        anglular_velocity = (self.wheelbase * angle_rad) / (
            2 * self.wheel_radius * self.time_for_rotation
        )

        wheel_velocities_rotate = WheelAngularVelocities()
        wheel_velocities_rotate.left_wheel_angular_velocity = -anglular_velocity
        wheel_velocities_rotate.right_wheel_angular_velocity = anglular_velocity
        self.wheel_velocity_publisher.publish(wheel_velocities_rotate)
        self.time_increment += 1
        # self.get_logger().info(f'current time that you should know of is:{self.time_increment/10}')

    def linear_control(self):

        distance = 1

        angular_velocity = distance / (self.wheel_radius * self.time_for_linear)
        wheel_velocities_rotate = WheelAngularVelocities()
        wheel_velocities_rotate.left_wheel_angular_velocity = angular_velocity
        wheel_velocities_rotate.right_wheel_angular_velocity = angular_velocity
        self.wheel_velocity_publisher.publish(wheel_velocities_rotate)
        self.time_increment += 1
        # self.get_logger().info(f'current time that you should know of is:{self.time_increment/10}')

    def robot_turn_off(self):
        while not self.service_client_robot_off.wait_for_service(timeout_sec=1):
            self.get_logger().info(
                f"{self.service_client_robot_off.srv_name} service is not yet available"
            )
        self.future_off: Future = None

        request_off = TurnRobotOff.Request()
        if self.future_off is not None and not self.future_off.done():
            self.future_off.cancel()
            self.get_logger().warn("Turn on cancelled is operator node still running? ")

        self.future_off = self.service_client_robot_off.call_async(request_off)
        self.future_off.add_done_callback(self.after_robot_off)

    def after_robot_off(self, future_off: Future):
        wheel_velocities_rotate = WheelAngularVelocities()
        wheel_velocities_rotate.left_wheel_angular_velocity = 0.0
        wheel_velocities_rotate.right_wheel_angular_velocity = 0.0
        self.wheel_velocity_publisher.publish(wheel_velocities_rotate)
        response = future_off.result()
        if response.success == True:
            self.get_logger().info("Robot off sequence has succeeded!")
        else:
            self.get_logger().info(
                f"Off sequence response is: {response.success} robot did not turn off successfully!"
            )


def main(args=None):
    try:
        rclpy.init(args=args)
        omega_operator_controller_node = OmegaOperatorControllerNode()
        rclpy.spin(omega_operator_controller_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
"/rplidar_node/scan/rvis2/initialpose"
