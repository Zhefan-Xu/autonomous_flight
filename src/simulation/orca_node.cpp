/*
	FILE: orca_node.cpp
	Execution file for orca: obstacle avoidance
*/

#include <ros/ros.h>
#include <autonomous_flight/simulation/quadcopter_command.h>
#include <reactive_planner/ORCAPlanner.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "orca_node");
	ros::NodeHandle nh;

	AutoFlight::quadCommand qm (nh);
	qm.takeoff();
	qm.resetPosition(5, 5, 1, 0);

	return 0;
}