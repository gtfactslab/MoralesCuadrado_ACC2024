# MoralesCuadrado_ACC2024
## Preliminary
1. Install and run PX4 SITL ([https://docs.px4.io/main/en/simulation/](https://docs.px4.io/main/en/ros/ros2.html)) with Gazebo classic
2. Create a ROS2 workspace with the px4_msgs package in name_ws/src. It is a needed dependency for ros2-px4 communication
3. Clone this stack as another package within this ROS2 workspace
4. In the root of the workspace, build everything as:
```
colcon build
```
5. Create a conda environment with simpy, scipy, and numpy

## Simulation Run and Communication
1. Run the SITL simulation
```
make px4_sitl gazebo-classic
```
2. Run the Micro XRCE Agent as the bridge between ROS2 and PX4.
```
MicroXRCEAgent udp4 -p 8888
```
Once this works, you're ready to try using the Newton-Raphson Controller in simulation.

## To Run The NR Controller Computation and Offboard Publisher
The reference path may be changed through the reffunc variable starting in line 139 of the nr_tracker_final.py file. New ones may be defined and referenced here.
1. If changes have been made to the reference, in one terminal, go to the root of your ROS2 workspace and build this package:
```
colcon build --packages-select Final_NR_Wardi_Tracker_Stack
```

### Through launch file:
1. In another terminal tab, source the environment from the root of your ROS2 workspace: 
```
source install/setup.bash
```
2. Activate your conda environment in this same terminal with sourcing
3. After sourcing and activating environment, run the launch file:
```
ros2 launch Final_NR_Wardi_Tracker_Stack nr_tracker_launch.py
```

### This can also be run by running the two separate files on their own:
1. In one terminal tab, source the environment as shown before, activate the environment and run:
```
ros2 run Final_NR_Wardi_Tracker_Stack nr_tracker_final.py
```
This is the computation file

2. In another terminal, source again, and run:
```
ros2 run Final_NR_Wardi_Tracker_Stack offboard_pub
```
This is the offboard publisher that takes the computed input and communicates it to the robot
