import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_ros.actions import Node


def generate_launch_description():
    pkg_rover_gazebo = get_package_share_directory('rover_gazebo')
    pkg_rover_perception = get_package_share_directory('rover_perception')
    pkg_rover_navigation = get_package_share_directory('rover_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    mode = LaunchConfiguration('mode')  # 'nav' or 'slam'
    run_perception = LaunchConfiguration('run_perception')
    run_mission = LaunchConfiguration('run_mission')
    map_yaml_file = LaunchConfiguration('map')
    output_dir = LaunchConfiguration('output_dir')
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    inspection_mode = LaunchConfiguration('inspection_mode')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_inspection_mode = DeclareLaunchArgument(
        'inspection_mode',
        default_value='baseline',
        description="Inspection mode: 'baseline' (fixed 7-waypoint sweep) or 'active' (NBV + coverage map)"
    )

    declare_mode = DeclareLaunchArgument(
        'mode',
        default_value='nav',
        description="Navigation mode: 'nav' (for autonomous navigation with Nav2) or 'slam' (for mapping)"
    )

    declare_run_perception = DeclareLaunchArgument(
        'run_perception',
        default_value='true',
        description='Launch defect detection and logging stack'
    )

    declare_run_mission = DeclareLaunchArgument(
        'run_mission',
        default_value='false',
        description='Automatically start the autonomous multi-waypoint mission manager'
    )

    default_map = os.path.join(pkg_rover_navigation, 'maps', 'facility.yaml')
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file to load (used in nav mode)'
    )

    declare_output_dir = DeclareLaunchArgument(
        'output_dir',
        default_value='results',
        description='Directory for defect CSV output (relative to cwd or absolute)'
    )

    default_world = os.path.join(pkg_rover_gazebo, 'worlds', 'inspection_facility.sdf')
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Full path to Gazebo SDF world file'
    )

    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode if true'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 if true'
    )

    default_rviz = os.path.join(pkg_rover_navigation, 'rviz', 'active_inspection_demo.rviz')
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz,
        description='Full path to RViz config file'
    )

    # 1. Gazebo Simulation + Rover + Bridge + Relay (scoped to prevent use_rviz leakage)
    sim_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_rover_gazebo, 'launch', 'sim.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'use_rviz': 'false',
                    'world': world,
                    'headless': headless
                }.items()
            )
        ],
        scoped=True
    )

    # 2. Perception Stack (Detector + Pose Projector + Defect Logger)
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_perception, 'launch', 'perception.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'output_dir': output_dir,
            'inspection_mode': inspection_mode
        }.items(),
        condition=IfCondition(run_perception)
    )

    # 3. SLAM Mode (slam_toolbox)
    slam_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_rover_navigation, 'launch', 'slam.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time
                }.items()
            )
        ],
        scoped=True,
        condition=LaunchConfigurationEquals('mode', 'slam')
    )

    # 4. Nav2 Mode (AMCL + Costmaps + Planner + Controller, scoped to prevent use_rviz leakage)
    nav_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_rover_navigation, 'launch', 'navigation.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': map_yaml_file,
                    'use_rviz': 'false',  # Launched directly as node_rviz below
                    'rviz_config': rviz_config
                }.items()
            )
        ],
        scoped=True,
        condition=LaunchConfigurationEquals('mode', 'nav')
    )

    # 5. Optional Mission Manager Node
    node_mission = Node(
        package='rover_mission',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'output_dir': output_dir,
            'world_name': world,
            'inspection_mode': inspection_mode,
        }],
        condition=IfCondition(run_mission)
    )

    # 6. RViz2 Visual Dashboard Node
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_mode,
        declare_run_perception,
        declare_run_mission,
        declare_inspection_mode,
        declare_map,
        declare_output_dir,
        declare_world,
        declare_headless,
        declare_use_rviz,
        declare_rviz_config,
        node_rviz,
        sim_launch,
        perception_launch,
        slam_launch,
        nav_launch,
        node_mission
    ])
