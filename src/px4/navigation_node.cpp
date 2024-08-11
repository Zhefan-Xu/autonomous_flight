/*
	FILE: navigation_node.cpp
	-----------------------------
	navigation ROS node
*/

#include <ros/ros.h>
#include <autonomous_flight/px4/navigation.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "navigation_node");
	ros::NodeHandle nh;

	AutoFlight::navigation navigator (nh);
	navigator.run();

	ros::spin();
	return 0;
}