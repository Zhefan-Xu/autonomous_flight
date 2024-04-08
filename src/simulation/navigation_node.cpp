/*
	FILE: navigation_node.cpp
	-----------------------------
	navigation ROS node
*/

#include <ros/ros.h>
#include <autonomous_flight/simulation/navigation.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "navigation_node");
	ros::NodeHandle nh;

	AutoFlight::navigation navigator (nh);
	navigator.run();
	ros::MultiThreadedSpinner spinner(2);
	spinner.spin();
	// ros::spin();
	return 0;
}