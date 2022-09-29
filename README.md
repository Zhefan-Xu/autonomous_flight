# autonomous_flight
This branch is for the customized simulation flight. Check the ```px4``` branch for real-world or PX4-based simulation.

### I. Autonomous Flight Options
  - ```Takeoff/Hovering```: Takeoff and hovering at the specified height. 
  - ```Navigation```: Autonomous navigation in static environment.  
  - ```Dynamic Navigation```: Autonomous navigation in dynamic environments.
  - ```Inspection (Octomap)```: Autonomous tunnel excavation frontier inspection using the Octomap.
  - ```Inspection (Dynamic Map)```: Autonomous Tunnel Inspection using the Dynamic Map. (In Progress)


### II. Installation
This package depends on [map_manager](https://github.com/Zhefan-Xu/map_manager), [global_planner](https://github.com/Zhefan-Xu/global_planner), [trajectory_planner](https://github.com/Zhefan-Xu/trajectory_planner) and [onboard_vision](https://github.com/Zhefan-Xu/onboard_vision). Please install those packages first!
```
cd ~/catkin_ws/src/CERLAB-Autonomy
git clone https://github.com/Zhefan-Xu/autonomous_flight.git

cd ~/catkin_ws
catkin_make
```

### III. Quick Start
```Takeoff/Hovering```:
```
todo
```
