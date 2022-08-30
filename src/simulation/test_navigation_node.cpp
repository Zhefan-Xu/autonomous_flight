/*
	FILE: test_navigation_node.cpp
	-----------------------------
	test navigation ROS node
*/

#include <ros/ros.h>
#include <autonomous_flight/simulation/testNavigation.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "test_navigation_node");
	ros::NodeHandle nh;

	AutoFlight::testNavigation navigator (nh);
	navigator.run();
	ros::spin();
	return 0;
}