# nuc_ws

![OS](https://img.shields.io/ubuntu/v/ubuntu-wallpapers/noble)
![ROS_2](https://img.shields.io/ros/v/jazzy/rclcpp)

ROS2 workspace for the NUC.

__Contents:__
1. [Installation](#installation)
    1. [ROS2 Environment Setup](#ros2-environment-setup)
2. [Usage](#usage)

### Installation
1. Clone this repository onto the NUC:
```
cd ~
```
```
git clone https://github.com/Team4-UoM-RSDP/nuc_ws
```
2. Navigate to the cloned workspace:
```
cd ~/nuc_ws
```
3. Install ROS2 dependencies:
```
rosdep update
```
```
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
```
3. Build and source the project:
```
colcon build --symlink-install
```
```
source ~/nuc_ws/install/setup.bash
```

#### ROS2 Environment Setup
To automatically setup the ROS2 environment on your cobot:
1. Open `.bashrc` for editing:
```
sudo nano ~/.bashrc
```
2. Add the following to the end of `.bashrc`:
```
# Source ROS Jazzy setup with error checking
if source /opt/ros/jazzy/setup.bash; then
  echo "Sourced /opt/ros/jazzy/setup.bash successfully"
else
  echo "Failed to source /opt/ros/jazzy/setup.bash"
fi

# Delcare the DDS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
echo "Explicilty set fast DDS"

# Export and print ROS_DOMAIN_ID
export ROS_DOMAIN_ID=4
echo "ROS_DOMAIN_ID is set to $ROS_DOMAIN_ID"
echo "To change this automation, use nano to edit ~/.bashrc and the source ~/.bashrc to apply."

# For quick setup use ROS_LOCALHOST_ONLY, do revise later
export ROS_LOCALHOST_ONLY=0
echo "ROS_LOCALHOST_ONLY set to $ROS_LOCALHOST_ONLY"
```
2. Save and exit with`CTRL+X`. Source to apply the changes:
```
source ~/.bashrc
```
### Usage

###### Team 4 – AERO62520 Robotic System Design Project 
