/*
	FILE: dynamic_inspection_node.cpp
	-----------------------------
	dynamic inspection ROS node
*/

#include <autonomous_flight/px4/dynamicInspection.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "dynamic_navigation_node");
	ros::NodeHandle nh;
	AutoFlight::dynamicInspection inspector (nh);
	inspector.run();

	ros::spin();

	return 0;
}