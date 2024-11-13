import os
from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
  ld = LaunchDescription()

  # Get configuration params file
  config = os.path.join(
    get_package_share_directory('bump_and_go'),
    'config',
    'config.yaml'
  )

  collision_detector_node = Node(
    package='bump_and_go',
    name='collision_detector_node',
    executable='collision_detector_node',
    parameters=[config]
  )
  ld.add_action(collision_detector_node)

  robot_mover_node = Node(
    package='bump_and_go',
    name='robot_mover_node',
    executable='robot_mover_node',
    parameters=[config]
  )
  ld.add_action(robot_mover_node)

  return ld