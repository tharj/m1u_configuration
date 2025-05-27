import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

    # Declare launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true. Set to false for visualizing URDF without Gazebo.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Flag to enable joint_state_publisher_gui'
        )
    )

    # Get launch configuration values
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')

    # Define paths
    pkg_m1u_desc = get_package_share_directory('m1u_desc')
    xacro_file = os.path.join(pkg_m1u_desc, 'xacro', 'm1u.xacro')
    rviz_config_file = os.path.join(pkg_m1u_desc, 'rviz', 'display.rviz')

    # Process xacro and load robot_description
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description_param = {'robot_description': robot_description_content}

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param, {'use_sim_time': use_sim_time}]
    )

    # Joint State Publisher GUI Node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=launch.conditions.IfCondition(gui), # Launch only if gui argument is true
        parameters=[{'use_sim_time': use_sim_time}] 
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription(declared_arguments + [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])