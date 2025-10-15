from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro
import os


def launch_setup(context, *args, **kwargs):
    """
    Function to process launch configurations and create nodes.
    This runs after all launch arguments are resolved.
    """
    # Get the actual values of launch configurations
    xacro_file_path = LaunchConfiguration('xacro_file').perform(context)
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    gui_str = LaunchConfiguration('gui').perform(context)
    rviz_str = LaunchConfiguration('rviz').perform(context)
    
    # Convert string to boolean
    use_sim_time = use_sim_time_str.lower() == 'true'
    
    # Process the xacro file to get the URDF XML as a string
    robot_description_config = xacro.process_file(xacro_file_path)
    robot_description = robot_description_config.toxml()
    
    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
    )
    
    # Joint state publisher GUI (conditional)
    nodes_to_start = [robot_state_publisher]
    
    if gui_str.lower() == 'true':
        joint_state_publisher_gui = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        )
        nodes_to_start.append(joint_state_publisher_gui)
    
    # RViz2 (conditional)
    if rviz_str.lower() == 'true':
        rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('rse_shl1_description'),
                'rviz',
                'display.rviz'
            ]).perform(context)],
            parameters=[{'use_sim_time': use_sim_time}],
        )
        nodes_to_start.append(rviz2)
    
    return nodes_to_start


def generate_launch_description():
    
    # Declare launch argument for the xacro file path
    xacro_file_arg = DeclareLaunchArgument(
        'xacro_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('rse_shl1_description'),
            'urdf',
            'shl1_robot.urdf.xacro'
        ]),
        description='Path to the robot xacro/urdf file'
    )
    
    # Declare launch argument for use_sim_time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Declare launch argument for showing GUI
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch joint_state_publisher_gui'
    )
    
    # Declare launch argument for RViz
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    return LaunchDescription([
        xacro_file_arg,
        use_sim_time_arg,
        gui_arg,
        rviz_arg,
        OpaqueFunction(function=launch_setup)
    ])