"""Launch file for the closed-loop 6-DOF swarm simulation and controller."""


from __future__ import annotations


from launch import LaunchDescription
from launch_ros.actions import Node


from drone_swarm import constants
from drone_swarm import launch_utils




def generate_launch_description() -> LaunchDescription:
  """Generates the full simulator + formation controller + RViz2 launch."""
  _, robot_desc = launch_utils.load_urdf_description()
  nodes: list[Node] = [
      launch_utils.create_robot_state_publisher_node(robot_desc, drone_name)
      for drone_name in constants.DRONE_NAMES
  ]


  nodes.append(
      Node(
          package=constants.PACKAGE_NAME,
          executable='swarm_sim',
          name='swarm_sim',
          output='screen',
      )
  )
  nodes.append(
      Node(
          package=constants.PACKAGE_NAME,
          executable='swarm_controller',
          name='swarm_controller',
          output='screen',
      )
  )
  nodes.append(launch_utils.create_rviz_node(constants.SWARM_RVIZ_FILENAME))
  return LaunchDescription(nodes)