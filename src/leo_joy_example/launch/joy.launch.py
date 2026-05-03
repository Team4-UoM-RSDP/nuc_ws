from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  ld = LaunchDescription()

  package_name = 'leo_joy_example'
  joy_config_path = os.path.join(
    get_package_share_directory(package_name),
    'config',
    'joy_mapping.yaml'
  )

  # Joy node
  joy_node = Node(
    package='joy',
    executable='joy_node',
    name='joy_node',
    output='screen',
    parameters=[{
      'dev': '/dev/input/js0',
      'coalesce_interval': 0.02,
      'autorepeat_rate': 30.0
    }]
  )

  # Teleop node
  teleop_node = Node(
    package='teleop_twist_joy',
    executable='teleop_node',
    name='teleop_node',
    output='screen',
    parameters=[joy_config_path],
    remappings=[('cmd_vel', 'cmd_vel')]
  )

  #manager node
  manager_node = Node(
    package='leo_joy_example',
    executable='manager_node',
    name='manager_node',
    output='screen',

    
  )

  # Add nodes to the launch description
  ld.add_action(joy_node)
  ld.add_action(teleop_node)
  ld.add_action(manager_node)

  return ld
