/*
	FILE: dynamic_exploration_node.cpp
	-----------------------------
	dynamic exploration ROS node
*/

#include <autonomous_flight/px4/dynamicExploration.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "dynamic_exploration_node");
	ros::NodeHandle nh;
	AutoFlight::dynamicExploration explorer (nh);
	explorer.run();

	ros::spin();

	return 0;
}