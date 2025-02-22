# MoralesCuadrado_ACC2024

Videos of the quadrotor flying may be seen ([here](https://www.youtube.com/watch?v=vQiQZZE2iDM))

## Preliminary
1. Follow the instructions ([here](https://docs.px4.io/main/en/ros/ros2_comm.html)) to set up the PX4 Autopilot Stack, ROS2, Micro XRCE-DDS Agent & Client, and build a ROS2 workspace with the necessary px4 communication repos
2. In the same workspace with the communication folders as above, go to the /src/ folder and clone this repository
3. In the root of the workspace, build everything as:
```
colcon build --symlink-install
```
5. Create a conda environment with simpy, scipy, and numpy

## Simulation Run and Communication (simulation)
1. Run the SITL simulation
```
make px4_sitl gazebo-classic
```
2. Run the Micro XRCE Agent as the bridge between ROS2 and PX4.
```
MicroXRCEAgent udp4 -p 8888
```


## Using The NR Controller Computation and Offboard Publisher
1. The length of time the algorithm runs before the land sequence begins may be changed via the variable in the **\_\_init\_\_** function at the top:   **self.time_before_land**
2. The reference path may be changed through the reffunc variable starting in line 400 of the nr_tracker_final.py file. New ones may be defined below in functions around line 646.
3. The mass for the quadrotor may be changed for your specific hardware on the **elif not self.sim** statement in the **\_\_init\_\_** at the top. Don't change for simulation unless you change the simulation model explicitly.
4. The thrust/throttle mapping may be changed for your specific hardware on the **get_throttle_command_from_force** and **get_force_from_throttle_command** functions. Don't change for simulation unless you change the simulation model explicitly.

### Running the controller:
1. In another terminal tab, source the environment from the root of your ROS2 workspace: 
```
source install/setup.bash
```
2. Activate your conda environment in this same terminal with sourcing
3. After sourcing and activating environment, run the file:
```
ros2 run newton_raphson_controller newton_raphson
```
4. When prompted, answer [0/1] whether in simulation or hardware.


## Citing this Work:
Please [cite this paper](https://ieeexplore.ieee.org/document/10644692) ([arxiv version here](https://arxiv.org/abs/2408.11197#:~:text=We%20apply%20the%20Newton%2DRaphson,in%20the%20popular%20PX4%20Autopilot.)).

## Authors:
Evanns G. Morales-Cuadrado, Christian Llanes, Yorai Wardi, Samuel Coogan


