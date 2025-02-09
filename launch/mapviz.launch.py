from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ros_gz_sim.actions import GzServer
from ros_gz_bridge.actions import RosGzBridge
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='heibjerg').find('heibjerg')
    mapviz_config_file = os.path.join(pkg_share, 'config', 'gps_wpf_demo.mvc')



    mapviz_node = Node(
        package="mapviz",
        executable="mapviz",
        name="mapviz",
        #parameters=[{"config": mapviz_config_file}]
    )

    something = Node(
        package="swri_transform_util",
        executable="initialize_origin.py",
        name="initialize_origin",
        parameters=[
            {"local_xy_frame": "map"},
            {"local_xy_origin": "swri"},
            {"local_xy_origins": """[
                {"name": "swri",
                    "latitude": 29.45196669,
                    "longitude": -98.61370577,
                    "altitude": 233.719,
                    "heading": 0.0},
                {"name": "back_40",
                    "latitude": 29.447507,
                    "longitude": -98.629367,
                    "altitude": 200.0,
                    "heading": 0.0}
            ]"""},
            ],
        remappings=[
            ("fix", "/navsat"),
        ],
    )

    tf2_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="swri_transform",
        arguments=["0", "0", "0", "0", "0", "0", "map", "origin"]
    )

    return LaunchDescription([
        mapviz_node,
        something,
        tf2_node  
    ])