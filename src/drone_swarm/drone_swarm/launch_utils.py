"""Reusable ROS 2 launch node builders for the drone_swarm package."""


from __future__ import annotations


from collections.abc import Sequence
import os


from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


from drone_swarm import constants




def get_asset_path(folder: str, filename: str) -> str:
  """Resolves an installed package asset path inside share/drone_swarm.


  Args:
    folder: Subdirectory name ('urdf' or 'config').
    filename: File basename inside the subdirectory.


  Returns:
    Absolute path to the installed file.
  """
  pkg_share = get_package_share_directory(constants.PACKAGE_NAME)
  return os.path.join(pkg_share, folder, filename)




def load_urdf_description(
    urdf_filename: str = constants.URDF_FILENAME,
) -> tuple[str, str]:
  """Reads the URDF XML string and returns (urdf_path, robot_description).


  Args:
    urdf_filename: Name of the URDF file inside share/drone_swarm/urdf.


  Returns:
    Tuple of (urdf_file_path, urdf_xml_contents).
  """
  urdf_path = get_asset_path('urdf', urdf_filename)
  with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
    return urdf_path, urdf_file.read()




def create_robot_state_publisher_node(
    robot_description: str,
    drone_namespace: str = '',
) -> Node:
  """Creates a robot_state_publisher Node (with optional frame_prefix).


  Args:
    robot_description: Raw URDF XML string.
    drone_namespace: Optional robot namespace (e.g. 'drone_1').


  Returns:
    Configured launch_ros Node action.
  """
  params: dict[str, str] = {'robot_description': robot_description}
  if drone_namespace:
    params['frame_prefix'] = f'{drone_namespace}/'


  return Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      namespace=drone_namespace,
      name='robot_state_publisher',
      parameters=[params],
      output='screen',
  )




def create_joint_state_publisher_node(
    urdf_path: str,
    drone_namespace: str = '',
) -> Node:
  """Creates a joint_state_publisher Node for static preview launch files.


  Args:
    urdf_path: Absolute path to the URDF file.
    drone_namespace: Optional robot namespace (e.g. 'drone_1').


  Returns:
    Configured launch_ros Node action.
  """
  return Node(
      package='joint_state_publisher',
      executable='joint_state_publisher',
      namespace=drone_namespace,
      name='joint_state_publisher',
      arguments=[urdf_path],
      output='screen',
  )




def create_static_world_tf_node(
    drone_namespace: str,
    position_xyz: Sequence[float],
    yaw_rad: float = 0.0,
) -> Node:
  """Creates a static_transform_publisher Node from world to drone/base_link.


  Args:
    drone_namespace: Robot namespace (e.g. 'drone_1').
    position_xyz: Static [x, y, z] coordinates in world frame (m).
    yaw_rad: Static yaw angle in radians.


  Returns:
    Configured launch_ros Node action.
  """
  child_frame = f'{drone_namespace}/{constants.BASE_LINK_NAME}'
  return Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name=f'{drone_namespace}_world_tf',
      arguments=[
          '--x', str(position_xyz[0]),
          '--y', str(position_xyz[1]),
          '--z', str(position_xyz[2]),
          '--roll', '0.0',
          '--pitch', '0.0',
          '--yaw', str(yaw_rad),
          '--frame-id', constants.WORLD_FRAME,
          '--child-frame-id', child_frame,
      ],
      output='screen',
  )




def create_rviz_node(
    rviz_filename: str = constants.SWARM_RVIZ_FILENAME,
) -> Node:
  """Creates an rviz2 Node pre-loaded with the specified .rviz config file.


  Args:
    rviz_filename: Name of the .rviz file in share/drone_swarm/config.


  Returns:
    Configured launch_ros Node action.
  """
  rviz_config_path = get_asset_path('config', rviz_filename)
  return Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      arguments=['-d', rviz_config_path],
      output='screen',
  )