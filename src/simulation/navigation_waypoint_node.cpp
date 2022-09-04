/*
	FILE: navigation_waypoint_node.cpp
	-----------------------------
	navigation ROS node
*/

#include <ros/ros.h>
#include <autonomous_flight/simulation/navigationWaypoint.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "navigation_waypoint_node");
	ros::NodeHandle nh;
	std::vector<double> waypoints;
	if (not nh.getParam("/waypoints", waypoints)){
		ROS_ERROR("No waypoints!");
		exit(0);
	}
 	

	geometry_msgs::PoseStamped ps;
	nav_msgs::Path path;
 	int waypointNum = waypoints.size()/3;
	for (int i=0; i<waypointNum; ++i){
		ps.pose.position.x = waypoints[3*i];
		ps.pose.position.y = waypoints[3*i+1];
		ps.pose.position.z = waypoints[3*i+2];
		path.poses.push_back(ps);
	}

	AutoFlight::navigationWaypoint navigator (nh);
	// navigator.updateWaypoints(path);
	navigator.run(path);
	ros::spin();
	return 0;
}