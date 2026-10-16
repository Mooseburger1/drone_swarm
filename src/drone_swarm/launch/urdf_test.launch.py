import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node




def generate_launch_description():
    pkg_share = get_package_share_directory('drone_swarm')
    urdf_path = os.path.join(pkg_share, 'urdf', 'quadrotor.urdf')


    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen',
    )


    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[urdf_path],
        output='screen',
    )


    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
    ])