import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node




def generate_launch_description():
    pkg_share = get_package_share_directory('drone_swarm')
    urdf_path = os.path.join(pkg_share, 'urdf', 'quadrotor.urdf')
    rviz_config_path = os.path.join(pkg_share, 'config', 'swarm.rviz')


    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()


    # Equilateral triangle formation (side length d* = 2.0 m, altitude z = 1.0 m)
    drones = [
        {'name': 'drone_1', 'x': '1.1547',  'y': '0.0',  'z': '1.0', 'yaw': '0.0'},
        {'name': 'drone_2', 'x': '-0.5774', 'y': '1.0',  'z': '1.0', 'yaw': '0.0'},
        {'name': 'drone_3', 'x': '-0.5774', 'y': '-1.0', 'z': '1.0', 'yaw': '0.0'},
    ]


    nodes = []


    for d in drones:
        ns = d['name']
        prefix = f"{ns}/"


        # 1. Robot State Publisher (publishes /drone_i/robot_description and prefixed link TFs)
        nodes.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=ns,
                name='robot_state_publisher',
                parameters=[{
                    'robot_description': robot_desc,
                    'frame_prefix': prefix,
                }],
                output='screen',
            )
        )


        # 2. Joint State Publisher (publishes /drone_i/joint_states for the 4 continuous rotors)
        nodes.append(
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                namespace=ns,
                name='joint_state_publisher',
                arguments=[urdf_path],
                output='screen',
            )
        )


        # 3. Static Transform Publisher: world -> drone_i/base_link
        nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'{ns}_world_tf',
                arguments=[
                    '--x', d['x'],
                    '--y', d['y'],
                    '--z', d['z'],
                    '--roll', '0.0',
                    '--pitch', '0.0',
                    '--yaw', d['yaw'],
                    '--frame-id', 'world',
                    '--child-frame-id', f'{ns}/base_link',
                ],
                output='screen',
            )
        )


    # 4. Launch RViz2 with config/swarm.rviz
    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
        )
    )


    return LaunchDescription(nodes)