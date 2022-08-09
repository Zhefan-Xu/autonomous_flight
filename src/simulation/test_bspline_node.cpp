/*
	FILE: test_bspline_node.cpp
	Execution file for test bspline
*/

#include <ros/ros.h>
#include <autonomous_flight/simulation/quadcopter_command.h>
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <trajectory_planner/bsplineTraj.h>
#include <map_manager/occupancyMap.h>
#include <visualization_msgs/MarkerArray.h>



int pointCount = 0;
visualization_msgs::MarkerArray getVisMsg(const nav_msgs::Path& path){
	visualization_msgs::MarkerArray msg;
	visualization_msgs::Marker point;
	std::vector<visualization_msgs::Marker> pointVec;
	for (size_t i=0; i<path.poses.size(); ++i){
		point.header.frame_id = "map";
		point.header.stamp = ros::Time::now();
		point.ns = "traj_points";
		point.id = pointCount;
		point.type = visualization_msgs::Marker::SPHERE;
		point.action = visualization_msgs::Marker::ADD;
		point.pose.position.x = path.poses[i].pose.position.x;
		point.pose.position.y = path.poses[i].pose.position.y;
		point.pose.position.z = path.poses[i].pose.position.z;
		point.lifetime = ros::Duration(0.5);
		point.scale.x = 0.3;
		point.scale.y = 0.3;
		point.scale.z = 0.3;
		point.color.a = 0.5;
		point.color.r = 0.0;
		point.color.g = 1.0;
		point.color.b = 0.0;
		pointVec.push_back(point);
		++pointCount;	
	}
	msg.markers = pointVec;
	return msg;
}

ros::Publisher pwlTrajPub;
visualization_msgs::MarkerArray pwlVisMsg;

void visCallback(const ros::TimerEvent&){
	pwlTrajPub.publish(pwlVisMsg);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "orca_node");
	ros::NodeHandle nh;
	ros::Timer visTimer = nh.createTimer(ros::Duration(0.1), visCallback);
	pwlTrajPub = nh.advertise<visualization_msgs::MarkerArray>("/pwl_traj", 10);



	AutoFlight::quadCommand qm (nh);
	qm.takeoff(); 

	// // click to get goal position
	std::vector<double> goal = qm.getGoal();
	std::vector<double> pos = qm.getPosition();
	nav_msgs::Path simplePath; // contains start and end
	geometry_msgs::PoseStamped pStart, pGoal;
	pStart.pose.position.x = pos[0];
	pStart.pose.position.y = pos[1];
	pStart.pose.position.z = pos[2];

	pGoal.pose.position.x = goal[0];
	pGoal.pose.position.y = goal[1];
	pGoal.pose.position.z = goal[2];
	std::vector<geometry_msgs::PoseStamped> pathVec {pStart, pGoal};
	simplePath.poses = pathVec;

	// // planners init
	trajPlanner::pwlTraj pwlPlanner (nh);

	// use planner to generate path
	double pwlTimeStep = 0.1;
	nav_msgs::Path simpleTraj;
	pwlPlanner.updatePath(simplePath);
	pwlPlanner.makePlan(simpleTraj, pwlTimeStep);
	pwlVisMsg = getVisMsg(simpleTraj);

	// bspline and  map 
	std::shared_ptr<mapManager::occMap> m (new mapManager::occMap);
	m->initMap(nh);

	ros::Rate r (20);
	ros::Time startTime, endTime;
	startTime = ros::Time::now();
	while (ros::ok()){
		endTime = ros::Time::now();
		if ((endTime - startTime).toSec() >= 2){
			break;
		}

		ros::spinOnce();
		r.sleep(); // wait for map to be stablized
	}

	trajPlanner::bsplineTraj bst;
	bst.init(nh);
	bst.setMap(m);
	std::vector<Eigen::Vector3d> startEndCondition; // dummy start end condition (all zeros)
	for (int i=0; i<4; ++i){
		Eigen::Vector3d con (0, 0, 0);
		startEndCondition.push_back(con);
	}
	bst.updatePath(simpleTraj, startEndCondition);
	bst.makePlan();

	ros::Rate rate (10);
	double t = 0.0;
	while (not qm.isReach(pGoal) and t <= bst.getDuration()){
		qm.setPose(bst.getPose(t));
		t += 0.1;
		rate.sleep();
	}


	ros::spin();

	return 0;
}