import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_rover_description = get_package_share_directory('rover_description')
    pkg_rover_gazebo = get_package_share_directory('rover_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Environment variables for Gazebo plugins and resources
    set_gz_plugin_path = AppendEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        '/opt/ros/jazzy/lib'
    )
    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_rover_description, '..')
    )

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')

    # Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    default_world_path = os.path.join(pkg_rover_gazebo, 'worlds', 'inspection_facility.sdf')
    declare_world = DeclareLaunchArgument(
        'world', default_value=default_world_path,
        description='Path to Gazebo SDF world file'
    )

    declare_x = DeclareLaunchArgument('x', default_value='-2.0', description='Initial X position')
    declare_y = DeclareLaunchArgument('y', default_value='-1.0', description='Initial Y position')
    declare_z = DeclareLaunchArgument('z', default_value='0.08', description='Initial Z position')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0', description='Initial Yaw orientation')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 if true'
    )

    declare_headless = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo in headless mode if true'
    )

    # Robot Description
    xacro_file = os.path.join(pkg_rover_description, 'urdf', 'rover.urdf.xacro')
    robot_description_content = Command([
        'xacro ', xacro_file,
        ' use_sim_time:=true',
        ' use_gazebo:=true'
    ])
    robot_description_param = ParameterValue(robot_description_content, value_type=str)

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_param,
            'use_sim_time': use_sim_time
        }]
    )

    # Gazebo Sim Launch
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world]
        }.items()
    )

    # Spawn Entity in Gazebo
    node_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'rover',
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw
        ]
    )

    # ros2_control Controller Spawners
    node_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30'],
        output='screen'
    )

    node_diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager-timeout', '30'],
        output='screen'
    )

    # ros_gz_bridge
    bridge_config_file = os.path.join(pkg_rover_gazebo, 'config', 'gz_bridge.yaml')
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config_file,
            'use_sim_time': use_sim_time
        }]
    )

    # Twist & Odometry Relay (bridges /cmd_vel <-> /diff_drive_controller/cmd_vel and odom)
    node_twist_relay = Node(
        package='rover_gazebo',
        executable='twist_relay.py',
        name='twist_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RViz2
    rviz_config_file = os.path.join(pkg_rover_description, 'rviz', 'rover.rviz')
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        set_gz_plugin_path,
        set_gz_resource_path,
        declare_use_sim_time,
        declare_world,
        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        declare_use_rviz,
        declare_headless,
        gz_sim_launch,
        node_robot_state_publisher,
        node_spawn_entity,
        node_joint_state_broadcaster_spawner,
        node_diff_drive_controller_spawner,
        node_ros_gz_bridge,
        node_twist_relay,
        node_rviz
    ])
