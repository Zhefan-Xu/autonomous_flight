/*
	FILE: dynamic_navigation_node.cpp
	-----------------------------
	dynamic navigation ROS node
*/

#include <ros/ros.h>
#include <autonomous_flight/simulation/dynamicNavigation.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "dynamic_navigation_node");
	ros::NodeHandle nh;

	AutoFlight::dynamicNavigation navigator (nh);
	navigator.run();
	ros::spin();
	return 0;
}