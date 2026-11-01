"""Launch file to preview a single quadrotor URDF in RViz2."""


from __future__ import annotations


from launch import LaunchDescription


from drone_swarm import constants
from drone_swarm import launch_utils




def generate_launch_description() -> LaunchDescription:
  """Generates the single-drone URDF preview launch description."""
  urdf_path, robot_desc = launch_utils.load_urdf_description()
  return LaunchDescription([
      launch_utils.create_robot_state_publisher_node(robot_desc),
      launch_utils.create_joint_state_publisher_node(urdf_path),
      launch_utils.create_rviz_node(constants.SINGLE_RVIZ_FILENAME),
  ])