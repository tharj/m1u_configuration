from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Declare the sim time argument
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time"
        )
    ]

    moveit_config = (
        MoveItConfigsBuilder("m1u_sim", package_name="m1u_moveit_conf")
        .to_moveit_configs()
    )

    # Override use_sim_time in the robot description
    moveit_config.robot_description["use_sim_time"] = use_sim_time
    moveit_config.robot_description_semantic["use_sim_time"] = use_sim_time
    moveit_config.robot_description_kinematics["use_sim_time"] = use_sim_time

    # Generate and return full LaunchDescription
    ld = generate_move_group_launch(moveit_config)

    # Add declared arguments to the launch description
    for arg in declared_arguments:
        ld.add_action(arg)

    return ld
