#include <ros/ros.h>
#include <autonomous_flight/px4/mpcNavigation.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "mpc_navigation_node");
	ros::NodeHandle nh;

	AutoFlight::mpcNavigation navigator (nh);
	navigator.run();
	ros::spin();
	return 0;
}