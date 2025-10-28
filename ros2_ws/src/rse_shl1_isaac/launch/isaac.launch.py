import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit  # OnProcessStart removed
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    """
    This function is called by OpaqueFunction after launch args are evaluated.
    It processes the xacro, defines all nodes, and returns them in a list.
    """
    # --- 1. Get and Process Launch Arguments ---
    
    # Get the xacro file path from the launch argument
    xacro_file_path = LaunchConfiguration('xacro_file').perform(context)

    # Get paths to controller config YAMLs
    control_pkg_share = get_package_share_directory('rse_shl1_control')
    swerve_controller_pkg_share = get_package_share_directory('rse_shl1_swerve_controller')

    controllers_yaml_path = os.path.join(control_pkg_share, 'config', 'controllers.yaml')
    swerve_controller_yaml_path = os.path.join(swerve_controller_pkg_share, 'config', 'swerve_controller.yaml')
    
    # Get the use_sim_time string and convert to boolean
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time_str.lower() == 'true'
    
    # Get the rviz string
    rviz_str = LaunchConfiguration('rviz').perform(context)

    # Process the xacro file to get the robot_description XML string
    robot_description_config = xacro.process_file(xacro_file_path)
    robot_description_xml = robot_description_config.toxml()

    
    # --- 2. Define Core ROS 2 Nodes ---

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_xml,
            'use_sim_time': use_sim_time
        }]
    )

    # Controller Manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_xml},
            {'use_sim_time': use_sim_time},
            controllers_yaml_path,           
            swerve_controller_yaml_path      
        ],
        output='screen',
    )
    
    # --- 3. Define Spawner Nodes ---
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    load_swerve_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['swerve_controller'],
        output='screen',
    )

    # --- 4. Define Event Handlers for Ordered Launch ---

    # Start the swerve controller spawner *after* the JSB spawner has finished
    load_swerve_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_swerve_controller],
        )
    )

    # --- 5. Define List of Nodes to Launch ---
    nodes_to_launch = [
        robot_state_publisher_node,
        controller_manager_node,
        load_joint_state_broadcaster,
        load_swerve_controller_event
    ]
    
    if rviz_str.lower() == 'true':
        # Get path to rviz config
        rviz_config_file = PathJoinSubstitution([
            FindPackageShare('rse_shl1_description'),
            'rviz',
            'display.rviz'
        ]).perform(context)
        
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
        )
        nodes_to_launch.append(rviz_node)

    return nodes_to_launch


def generate_launch_description():
    
    # Get the package share directory for default paths
    description_pkg_share = FindPackageShare('rse_shl1_description')

    # --- 1. Declare Launch Arguments ---
    
    xacro_file_arg = DeclareLaunchArgument(
        'xacro_file',
        default_value=PathJoinSubstitution([
            description_pkg_share,
            'urdf',
            'shl1_robot_isaac.urdf.xacro'
        ]),
        description='Path to the robot xacro/urdf file for Isaac Sim'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    # --- 2. Return LaunchDescription ---
    # The OpaqueFunction defers the execution of launch_setup
    # until after all launch arguments are resolved.
    
    return LaunchDescription([
        xacro_file_arg,
        use_sim_time_arg,
        rviz_arg,
        OpaqueFunction(function=launch_setup)
    ])