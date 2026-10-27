import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_rover_navigation = get_package_share_directory('rover_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    default_params_file = os.path.join(
        pkg_rover_navigation, 'config', 'slam_toolbox_online_async.yaml'
    )
    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_params_file,
        description='Path to slam_toolbox parameter YAML'
    )

    node_slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params,
        node_slam_toolbox
    ])
