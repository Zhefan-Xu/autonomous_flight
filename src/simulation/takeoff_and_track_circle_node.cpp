/*
	FILE: takeoff_and_track_circle_node.cpp
	-----------------------------
	navigation ROS node
*/

#include <ros/ros.h>
#include <autonomous_flight/simulation/flightBase.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "takeoff_and_track_circle_node");
	ros::NodeHandle nh;

	AutoFlight::flightBase fb (nh);
	fb.takeoff();
	fb.run();
	ros::spin();
	return 0;
}