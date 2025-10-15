import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare launch arguments
    xacro_file_arg = DeclareLaunchArgument(
        'xacro_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('rse_shl1_description'),
            'urdf',
            'shl1_robot_gazebo.urdf.xacro'
        ]),
        description='Path to the robot xacro/urdf file'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='https://fuel.gazebosim.org/1.0/MovAi/worlds/tugbot_depot',
        description='World file to load in Gazebo'
    )
    
    # Get launch configurations
    xacro_file = LaunchConfiguration('xacro_file')
    world = LaunchConfiguration('world')

    # Get the path to the packages
    gazebo_pkg_share = get_package_share_directory('rse_shl1_gazebo')
    description_pkg_share = get_package_share_directory('rse_shl1_description')
    
    # Set the GZ_SIM_RESOURCE_PATH environment variable
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(description_pkg_share, '..'),
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ]
    )
    
    # Launch Gazebo Fortress
    gz_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        ])),
        launch_arguments={
             'gz_args': ['-r ', world]
        }.items()
    )

    # Launch robot_state_publisher from description package
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('rse_shl1_description'), 'launch', 'display.launch.py'
        ])),
        launch_arguments={
            'xacro_file': xacro_file,
            'use_sim_time': 'true',
            'gui': 'false',
            'rviz': 'true'
        }.items()
    )

    # Spawn the robot using ros_gz_sim/create
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        arguments=[
            '-name', 'SHL-1',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
        ],
        output='screen',
    )

    # Load joint_state_broadcaster
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    # Load swerve controller
    load_swerve_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['swerve_controller'],
        output='screen',
    )

    # Load joint_state_broadcaster after robot is spawned
    load_joint_state_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    # Load swerve controller after joint_state_broadcaster
    load_swerve_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_swerve_controller],
        )
    )

    return LaunchDescription([
        xacro_file_arg,
        world_arg,
        gz_resource_path,
        gz_gazebo,
        description_launch,
        spawn_entity,
        load_joint_state_broadcaster_event,
        load_swerve_controller_event,
    ])