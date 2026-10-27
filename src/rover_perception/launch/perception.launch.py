import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_perception')
    default_detector_params = os.path.join(pkg_share, 'config', 'detector_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    output_dir = LaunchConfiguration('output_dir')
    detector_params_file = LaunchConfiguration('detector_params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_output_dir = DeclareLaunchArgument(
        'output_dir',
        default_value='results',
        description='Directory for defect CSV output (relative to cwd or absolute). Created if missing.'
    )

    declare_detector_params = DeclareLaunchArgument(
        'detector_params_file',
        default_value=default_detector_params,
        description='Full path to detector parameters YAML'
    )

    node_detector = Node(
        package='rover_perception',
        executable='defect_detector',
        name='defect_detector',
        output='screen',
        parameters=[
            detector_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    node_projector = Node(
        package='rover_perception',
        executable='pose_projector',
        name='pose_projector',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    node_logger = Node(
        package='rover_perception',
        executable='defect_logger',
        name='defect_logger',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'output_dir': output_dir,
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_output_dir,
        declare_detector_params,
        node_detector,
        node_projector,
        node_logger
    ])
