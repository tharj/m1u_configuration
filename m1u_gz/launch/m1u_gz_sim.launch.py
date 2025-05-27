import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Declare launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",  # For RViz
            default_value="true", 
            description="Start RViz2 GUI."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_gui", # For Gazebo's GUI
            default_value="true", 
            description="Start Gazebo GUI."
        )
    )

    # Configure launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_gui_config = LaunchConfiguration("gui") 
    gazebo_gui_config = LaunchConfiguration("gazebo_gui")

    # Define paths
    pkg_m1u_gz = get_package_share_directory("m1u_gz")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    xacro_file = os.path.join(pkg_m1u_gz, "xacro", "m1u_sim.xacro")
    robot_description_content = Command(["xacro ", xacro_file]) 
    robot_description_param = {"robot_description": robot_description_content}

    rviz_config_file = os.path.join(pkg_m1u_gz, "rviz", "simulation.rviz")

    # Gazebo Sim with GUI
    gazebo_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": " -r -v 3 empty.sdf"}.items(),
        condition=IfCondition(gazebo_gui_config) 
    )

    # Gazebo Sim Headless
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": " --headless-rendering -s -r -v 3 empty.sdf"}.items(), 
        condition=UnlessCondition(gazebo_gui_config)
    )
    
    # Explicit Clock Bridge from Gazebo to ROS
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], # Bridge Gz Clock to ROS /clock
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}], 
    )

    # Spawn robot
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", "m1u",
            "-allow_renaming", "true",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param, {"use_sim_time": use_sim_time}],
    )

    # Controller Spawners
    # These talk to the controller_manager running inside the Gazebo plugin.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}], 
    )

    m1u_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["m1u_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}], 
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz_gui_config), 
    )

    return LaunchDescription(
        declared_arguments
        + [
            gazebo_with_gui,
            gazebo_headless,
            clock_bridge, # Explicit clock bridge included
            spawn_entity,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner, # Spawners included
            m1u_arm_controller_spawner,    # Spawners included
            rviz_node,
        ]
    )