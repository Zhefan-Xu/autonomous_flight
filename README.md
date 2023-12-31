# Autonomous Flight for Unmanned Aerial Vehicles (UAVs)

This package includes the autonomous flight functions for unmanned aerial vehicles (UAVs) implemented in C++/ROS. It integrates our  perception, planning, and control modules to achive UAV autonomy in different tasks.

- The branch ```px4``` is for the **real-world** or **PX4-based simulation** experiments. 
- The branch ```simulation``` is for **simulation** in our customized and easy-to-use simulator.

**Author**: [Zhefan Xu](https://zhefanxu.com/), Computational Engineering & Robotics Lab (CERLAB) at Carnegie Mellon University (CMU).

If you find this work helpful, kindly show your support by giving us a free ⭐️. Your recognition is truly valued.

This repo can be used as a standalone package and also comes as a module of our [autonomy framework](https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy).

## I. Autonomous Flight Options
  - ```Takeoff/Hovering```: Takeoff and hovering at the specified height.
  - ```Takeoff/Tracking Circle```: Takeoff and tracking a circular trajectory with the given radius and velocity. 
  - ```Navigation```: Autonomous navigation in static environments.  
  - ```Dynamic Navigation```: Autonomous navigation in dynamic environments.
  - ```Inspection (Octomap)```: Autonomous surface inspection using the Octomap.
  - ```Inspection (Dynamic Map)```: Autonomous surface inspection using the Dynamic Map.
  - ```Dynamic Exploration```: Autonomous exploration in unknown and dynamic environments. 


## II. Installation Guide
This package has been tested on Ubuntu 18.04/20.04 LTS with ROS Melodic/Noetic on a regular PC/laptop, NVIDIA Jetson Xavier NX, Orin NX and Intel NUC. Make sure you have installed the compatible ROS version.

This package depends on [onboard_detector](https://github.com/Zhefan-Xu/onboard_detector), [global_planner](https://github.com/Zhefan-Xu/global_planner), [trajectory_planner](https://github.com/Zhefan-Xu/trajectory_planner), [map_manager](https://github.com/Zhefan-Xu/map_manager), [tracking_controller](https://github.com/Zhefan-Xu/tracking_controller.git)  and [time_optimizer](https://github.com/Zhefan-Xu/time_optimizer.git). Please install those packages first!


```
cd ~/catkin_ws/src
git clone https://github.com/Zhefan-Xu/autonomous_flight.git

cd ~/catkin_ws
catkin_make
```

## III. Quick Start
Start the uav simulator first and you should be able to see a quadcopter in a predefined gazebo environment. Please check the repo [uav_simulator](https://github.com/Zhefan-Xu/uav_simulator) for further details.

To visualize the flight data, we provide the rviz launch files in this [repo](https://github.com/Zhefan-Xu/remote_control) for convenience.
```
# if you want to test in PX4-based simulator
roslaunch uav_simulator px4_start.launch

# if you want to test in our customized uav simulator
roslaunch uav_simulator start.launch
```

a. Simple takeoff and hovering ```Takeoff/Hovering```:
```
roslaunch autonomous_flight takeoff_and_hover.launch  # takeoff at 1.0 meter height (by default parameter)
```
b. Takeoff and tracking a circle ```Takeoff/Tracking Circle```:
```
roslaunch autonomous_flight takeoff_and_track_circle.launch  # takeoff and tracking a circular trajectory
```
c. Static environment navigation ```Navigation```:
```
roslaunch autonomous_flight navigation.launch   # navigation in the static enviornments
```
d. Dynamic environment navigation ```Dynamic Navigation```:
```
roslaunch autonomous_flight dynamic_navigation.launch     # navigation in the dynamic environments
```
e. Static environmnet inspection using octomap ```Inspection (Octomap)```:
```
roslaunch octomap_server octomap_mapping.launch # please remember to modify the parameters for ros topics/transforms
roslaunch autonomous_flight inspection.launch
```
f. Dynamic environment inspection using dynamic map ```Inspection (Dynamic Map)```:
```
roslaunch autonomous_flight dynamic_inspection.launch    # autonomous surface inspection
```
g. Unknown and dynamic environment exploration  ```Dynamic Exploration```:
```
roslaunch autonomous_flight dynamic_exploration.launch    # autonomous exploration in unknown and dynamic environments
```


## IV. Parameter Adjustment
All the parameters are in ```autonomous_flight/cfg``` folder. For example, the parameters for Dynamic Navigation can be found in autonomous_flight/cfg/dynamic_navgation/***.yaml




