# Autonomous Flight for Unmanned Aerial Vehicles (UAV)

This package includes the autonomous flight implementation for the PX4-based unmanned aerial vehicles. This branch is for the real-world or PX4-based simulation flight. Check the ```main``` branch for our customized simulation.


### I. Autonomous Flight Options
  - ```Takeoff/Hovering```: Takeoff and hovering at the specified height. 
  - ```Navigation```: Autonomous navigation in static environment.  
  - ```Dynamic Navigation```: Autonomous navigation in dynamic environments.
  - ```Inspection (Octomap)```: Autonomous tunnel excavation frontier inspection using the Octomap.
  - ```Inspection (Dynamic Map)```: Autonomous Tunnel Inspection using the Dynamic Map. (In Progress)


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
d. Static environmnet inspection using octomap ```Inspection (Octomap)``` (Only support in PX4 branch)

e. Dynamic environment inspection using dynamic map ```Inspection (Dynamic Map)```
```
roslaunch autonomous_flight dynamic_inspection.launch
```

### IV. Parameter Tunning
All the parameters are in ```autonomous/cfg``` folder. For example, the parameters for ```Dynamic Navigation``` can be found in ```autonomous_flight/cfg/dynamic_navgation/***.yaml```.
