/*
	FILE: takeoff_and_hover_node.cpp
	---------------------------
	Simple flight test for autonomous flight
*/

#include <ros/ros.h>
#include <autonomous_flight/px4/flightBase.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "takeoff_and_hover_node");
	ros::NodeHandle nh;

	AutoFlight::flightBase fb (nh);
	fb.takeoff();
	ros::spin();

	return 0;
}