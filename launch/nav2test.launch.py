from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ros_gz_sim.actions import GzServer
from ros_gz_bridge.actions import RosGzBridge
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from nav2_common.launch import RewrittenYaml

from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='heibjerg').find('heibjerg')

    rl_params = os.path.join(pkg_share, 'config/robot_localization.yaml')
    default_model_path = os.path.join(pkg_share, 'src', 'description', 'heibjerg_robot.sdf')
    bridge_config_path = os.path.join(pkg_share, 'config', 'bridge_config.yaml')
    world_path = os.path.join(pkg_share, 'world', 'my_world.sdf')
    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_no_map_params.yaml')

    configured_params = RewrittenYaml(source_file=nav2_params_path, root_key="", param_rewrites="", convert_types=True)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, 
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    gz_server = GzServer(
    world_sdf_file=world_path,
    container_name='ros_gz_container',
    create_own_container='True',
    use_composition='True',
    )

    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=bridge_config_path,
        container_name='ros_gz_container',
        create_own_container='False',
        use_composition='True',
    )

    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                'launch',
                                'gz_spawn_model.launch.py'])]),
        launch_arguments={
            'world': 'my_world',
            'topic': '/robot_description',
            'entity_name': 'heibjerg_bot',
            'x': '0.0',
            'y': '0.0',
            'z': '0.5',
        }.items(),
    )

    robot_localization_odom = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[rl_params, 
                   {'use_sim_time': LaunchConfiguration('use_sim_time')},
                   {'print_diagnostics': True}],
        remappings=[("odometry/filtered", "odometry/local")]
    )

    robot_localization_map = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[rl_params, 
                   {'use_sim_time': LaunchConfiguration('use_sim_time')},
                   {'print_diagnostics': True}],
        remappings=[("odometry/filtered", "odometry/global")]
    )

    Navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[rl_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('imu/data', 'imu'),
            ('gps/fix', '/navsat'),
            ('gps/filtered', 'gps/filtered'),
            ('odometry/gps', 'odometry/gps'),
            ('odometry/filtered', 'odometry/global')
        ]
    )


    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('nav2_bringup'),
                                   'launch',
                                   'navigation_launch.py'])]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': configured_params,
            'autostart': 'True',
        }.items()
    )

    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('nav2_bringup'),
                                   'launch',
                                   'rviz_launch.py'])]),
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen'),
        gz_server,
        ros_gz_bridge,
        spawn_entity,
        robot_state_publisher_node,
        robot_localization_odom,
        robot_localization_map,
        Navsat_transform,
        navigation2_cmd,
        rviz_node
    ])