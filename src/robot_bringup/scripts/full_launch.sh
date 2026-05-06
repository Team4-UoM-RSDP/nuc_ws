#!/bin/bash
# Single script to launch the mycobot with Gazebo, RViz, and MoveIt 2

cleanup() {
    echo "Cleaning up..."
    sleep 5.0
    pkill -9 -f "ros2|gazebo|gz|nav2|amcl|bt_navigator|nav_to_pose|rviz2|assisted_teleop|cmd_vel_relay|robot_state_publisher|joint_state_publisher|move_to_free|mqtt|autodock|cliff_detection|moveit|move_group|basic_navigator"
}

# Set up cleanup trap
trap 'cleanup' SIGINT SIGTERM

sleep 15
ros2 launch cobot_controller cobot_control.launch.py  &
echo "Launching COBOT node..."
sleep 15 
ros2 launch leo_joy_example joy.launch.py &
echo "Launching Joycon node..."
sleep 15
ros2 run object_detection object_detect &
echo "Launching object detection node..."
sleep 15
ros2 launch leo_exploration exploration_launch.py &
echo "Launching slam and nav node..."


echo "Adjusting camera position..."
# Keep the script running until Ctrl+C
wait