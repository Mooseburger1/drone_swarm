"""Launch file for the 50 Hz 6-DOF quadrotor swarm physics simulator."""


from __future__ import annotations


from launch import LaunchDescription
from launch_ros.actions import Node


from drone_swarm import constants
from drone_swarm import launch_utils




def generate_launch_description() -> LaunchDescription:
  """Generates the 6-DOF swarm simulator + RViz2 launch description."""
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
  nodes.append(launch_utils.create_rviz_node(constants.SWARM_RVIZ_FILENAME))
  return LaunchDescription(nodes)