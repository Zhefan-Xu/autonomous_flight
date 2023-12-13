# Autonomous Flight for Unmanned Aerial Vehicles (UAV)

This package includes the autonomous flight functions for unmanned aerial vehicles (UAVs) implemented in C++/ROS. It integrates perception, planning, and control modules to make UAV achieve autonomy in different tasks.

- The branch ```px4``` is for the **real-world** or **PX4-based simulation** experiments. 
- The branch ```simulation``` branch for **simulation** in our customized and easy-to-use simulator.


### I. Autonomous Flight Options/Tasks Descriptions
  - ```Takeoff/Hovering```: Takeoff and hovering at the specified height.
  - ```Takeoff/Track Circle```: Takeoff and tracking a circle trajectory with the given radius and velocity. 
  - ```Navigation```: Autonomous navigation in static environments.  
  - ```Dynamic Navigation```: Autonomous navigation in dynamic environments.
  - ```Inspection (Octomap)```: Autonomous surface inspection using the Octomap.
  - ```Inspection (Dynamic Map)```: Autonomous surface inspection using the Dynamic Map.
  - ```Dynamic Exploration```: Autonomous exploration in unknown and dynamic environments. 


### II. Installation
This full package depends on [uav_simulator](https://github.com/Zhefan-Xu/uav_simulator), [map_manager](https://github.com/Zhefan-Xu/map_manager), [global_planner](https://github.com/Zhefan-Xu/global_planner), [trajectory_planner](https://github.com/Zhefan-Xu/trajectory_planner) and [onboard_vision](https://github.com/Zhefan-Xu/onboard_vision). Please install those packages first!
```
cd ~/catkin_ws/src/CERLAB-Autonomy
git clone https://github.com/Zhefan-Xu/autonomous_flight.git

cd ~/catkin_ws
catkin_make
```

### III. Quick Start
Start the uav simulator first and you should be able to see a quadcopter in a predefined gazebo environment. Please check the repo [uav_simulator](https://github.com/Zhefan-Xu/uav_simulator) for further details.
```
roslaunch uav_simulator start.launch
```

a. Simple takeoff and hovering ```Takeoff/Hovering```:
```
rosrun autonomous_flight takeoff_and_hover_node  # takeoff at 1.0 meter height
```

b. Static environment navigation ```Navigation```
```
roslaunch autonomous_flight navigation.launch   # navigation in the static enviornment
```
c. Dynamic environment navigation ```Dynamic Navigation```
```
roslaunch autonomous_flight dynamic_navigation.launch     # navigation in the dynamic environment
```
d. Static environmnet inspection using octomap ```Inspection (Octomap)``` 
```
roslaunch octomap_server octomap_mapping.launch # remember to modify the parameters
roslaunch autonomous_flight inspection.launch
```

e. Dynamic environment inspection using dynamic map ```Inspection (Dynamic Map)```
```
roslaunch autonomous_flight dynamic_inspection.launch
```

### IV. Parameter Tunning
All the parameters are in ```autonomous/cfg``` folder. For example, the parameters for ```Dynamic Navigation``` can be found in ```autonomous_flight/cfg/dynamic_navgation/***.yaml```.


### TODO: Navigation
1. add flight speed parameter to navigation
2. parameters for whether use global planner
3. Target sending callback (yaw angle?)
4. All planners in one callback function for replan (global planner not do replan every time)
5. Collision checking failsafe (if trajectory has collision, stop motion)
