/*
	FILE: flight_test_node.cpp
	---------------------------
	Simple flight test for autonomous flight
*/

#include <ros/ros.h>
#include <autonomous_flight/px4/flightBase.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "flight_test_node");
	ros::NodeHandle nh;

	AutoFlight::flightBase fb (nh);
	fb.takeoff();
	fb.run();
	// fb.land();

	return 0;
}