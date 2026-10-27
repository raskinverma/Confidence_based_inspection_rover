from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    output_dir = LaunchConfiguration('output_dir')

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

    node_detector = Node(
        package='rover_perception',
        executable='defect_detector',
        name='defect_detector',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
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
        node_detector,
        node_projector,
        node_logger
    ])
