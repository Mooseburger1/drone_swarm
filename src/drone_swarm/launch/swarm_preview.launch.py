"""Launch file to preview the 3-drone static triangle formation in RViz2."""


from __future__ import annotations


from launch import LaunchDescription
from launch_ros.actions import Node


from drone_swarm import constants
from drone_swarm import launch_utils




def generate_launch_description() -> LaunchDescription:
  """Generates the multi-drone static formation preview launch description."""
  urdf_path, robot_desc = launch_utils.load_urdf_description()
  nodes: list[Node] = []


  for drone_name in constants.DRONE_NAMES:
    position_xyz = constants.EQUILIBRIUM_POSITIONS[drone_name]
    nodes.append(
        launch_utils.create_robot_state_publisher_node(robot_desc, drone_name)
    )
    nodes.append(
        launch_utils.create_joint_state_publisher_node(urdf_path, drone_name)
    )
    nodes.append(
        launch_utils.create_static_world_tf_node(drone_name, position_xyz)
    )


  nodes.append(launch_utils.create_rviz_node(constants.SWARM_RVIZ_FILENAME))
  return LaunchDescription(nodes)