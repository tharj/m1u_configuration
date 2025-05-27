## How to use?

### Prerequisites
Ubuntu 24.04 with ROS2 Jazzy was used to create and test this configuration

1. Install ROS2 Jazzy (https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
2. Install MoveIt2 (https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html)
3. Install Gazebo (https://gazebosim.org/docs/latest/ros_installation/)

### M1u configuration
1. Clone the packages from this repository into your ROS2 workspace
   - The ws_moveit workspace that is created in the MoveIt2 installation guide was used, and is used as path in the files
2. Build the packages
   
   ```
   source /opt/ros/jazzy/setup.bash
   colcon build --packages-select m1u_desc m1u_moveit_conf m1u_gz
   ```
4. Source the the workspace and launch the simulation
   ```
   source install/setup.bash
   ros2 launch m1u_gz moveit_gz.launch.py
   ```
This should launch both gazebo simulation of the robot, and a RViz window with MoveIt2.

### Gazebo path problem
I had problems with gazebo not finding the mesh files, the paths were set manually by running the following
    
```
M1U_DESC_SHARE_FOR_GZ="$(ros2 pkg prefix m1u_desc)/share"
export GZ_SIM_RESOURCE_PATH="${M1U_DESC_SHARE_FOR_GZ}${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export IGN_GAZEBO_RESOURCE_PATH="${M1U_DESC_SHARE_FOR_GZ}${IGN_GAZEBO_RESOURCE_PATH:+:${IGN_GAZEBO_RESOURCE_PATH}}"
```

The gazebo resource paths can be checked with 

```
echo "GZ_SIM_RESOURCE_PATH is now: $GZ_SIM_RESOURCE_PATH"
echo "IGN_GAZEBO_RESOURCE_PATH is now: $IGN_GAZEBO_RESOURCE_PATH"
```

https://github.com/user-attachments/assets/591bd2f8-aef9-400e-b335-6672a6a1598a

