import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Declare top-level launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true", # Critical for Gazebo + MoveIt
            description="Use simulation (Gazebo) clock if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz", # Changed from 'gui' to be more specific for RViz
            default_value="true", 
            description="Start RViz2 with MoveIt configuration."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_gazebo_gui", # Changed from 'gazebo_gui' for clarity
            default_value="true", 
            description="Start Gazebo GUI."
        )
    )

    # Configure launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz_config = LaunchConfiguration("start_rviz")
    start_gazebo_gui_config = LaunchConfiguration("start_gazebo_gui")

    # Define paths
    pkg_m1u_gz = get_package_share_directory("m1u_gz")
    pkg_m1u_moveit_conf = get_package_share_directory("m1u_moveit_conf") # Path to your MoveIt config package
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Robot Description (from m1u_gz, includes ros2_control for Gazebo)
    xacro_file = os.path.join(pkg_m1u_gz, "xacro", "m1u_sim.xacro")
    robot_description_content = Command(["xacro ", xacro_file])
    robot_description_param = {"robot_description": robot_description_content}

    # RViz configuration file from MoveIt package (MSA usually generates a good one)
    # Or you can use your custom one from m1u_gz if preferred, but MoveIt's is pre-configured
    rviz_config_file_moveit = os.path.join(pkg_m1u_moveit_conf, "config", "moveit.rviz")


    # --- Gazebo Simulation Setup ---
    gazebo_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": " -r -v 3 empty.sdf"}.items(),
        condition=IfCondition(start_gazebo_gui_config) 
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": " --headless-rendering -s -r -v 3 empty.sdf"}.items(),
        condition=UnlessCondition(start_gazebo_gui_config)
    )
    clock_bridge = Node( # Ensure clock is available for all ROS nodes
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}], 
    )
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "/robot_description", "-name", "m1u", "-allow_renaming", "true"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # --- ROS 2 Controllers for Gazebo (Managed by gz_ros2_control) ---
    # These spawners start the controllers defined in m1u_gz/config/m1u_controllers.yaml
    # Ensure m1u_controllers.yaml defines JointTrajectoryController for the arm.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}], 
    )
    m1u_controller_spawner = Node( # This should now be spawning your JointTrajectoryController
        package="controller_manager",
        executable="spawner",
        arguments=["m1u_controller", "--controller-manager", "/controller_manager"], # Ensure 'm1u_arm_controller' is the JTC
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}], 
    )

    # --- MoveIt Setup ---
    # Robot State Publisher (Often started by MoveIt's launch files too, ensure only one is primary)
    # MoveIt's typical setup (e.g., via demo.launch.py or a MoveItConfigsBuilder approach) 
    # would start its own RSP. For simplicity here, we ensure one is running.
    # If m1u_moveit_conf/launch/move_group.launch.py starts RSP, you can comment this one out.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param, {"use_sim_time": use_sim_time}],
    )

    # MoveGroup node
    # We include move_group.launch.py from your MoveIt config package.
    # It needs use_sim_time and should NOT use fake_hardware.
    move_group_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_m1u_moveit_conf, "launch", "move_group.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            # Add other arguments that move_group.launch.py might expect or need overriding.
            # e.g., MSA might generate a 'use_fake_hardware' arg that needs to be false.
            # Check the DeclareLaunchArgument in m1u_moveit_conf/launch/move_group.launch.py
            # For now, assuming it picks up use_sim_time.
        }.items(),
    )

    # RViz with MoveIt panel
    # We include moveit_rviz.launch.py from your MoveIt config package.
    moveit_rviz_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_m1u_moveit_conf, "launch", "moveit_rviz.launch.py")
        ),
        launch_arguments={
            "rviz_config": rviz_config_file_moveit, # Use the RViz config from MoveIt
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(start_rviz_config),
    )

    return LaunchDescription(
        declared_arguments + [
            gazebo_with_gui,
            gazebo_headless,
            clock_bridge,
            spawn_entity,
            robot_state_publisher_node, # Keep one RSP
            joint_state_broadcaster_spawner,
            m1u_controller_spawner,
            move_group_launch_include,
            moveit_rviz_launch_include,
        ]
    )