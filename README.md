# Autonomous Flight for Unmanned Aerial Vehicles (UAV)

This package includes the autonomous flight implementation for the PX4-based unmanned aerial vehicles. This branch is for the real-world or PX4-based simulation flight. Check the ```main``` branch for our customized simulation.


### I. Autonomous Flight Options
  - ```Takeoff/Hovering```: Takeoff and hovering at the specified height. 
  - ```Navigation```: Autonomous navigation in static environment.  
  - ```Dynamic Navigation```: Autonomous navigation in dynamic environments.
  - ```Inspection (Octomap)```: Autonomous tunnel excavation frontier inspection using the Octomap.
  - ```Inspection (Dynamic Map)```: Autonomous Tunnel Inspection using the Dynamic Map. (In Progress)


### II. Quick Start
This package depends on [map_manager](https://github.com/Zhefan-Xu/map_manager), [global_planner](https://github.com/Zhefan-Xu/global_planner), [trajectory_planner](https://github.com/Zhefan-Xu/trajectory_planner) and [onboard_vision](https://github.com/Zhefan-Xu/onboard_vision). Please install those packages first!
