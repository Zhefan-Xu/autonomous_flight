/*
	FILE: navigationWaypoint.h
	------------------------
	navigation through waypoint header file in simulation
*/

#ifndef AUTOFLIGHT_NAVIGATION_WAYPOINT_H
#define AUTOFLIGHT_NAVIGATION_WAYPOINT_H

#include <autonomous_flight/simulation/flightBase.h>
#include <map_manager/occupancyMap.h>
// #include <map_manager/dynamicMap.h>
#include <onboard_vision/fakeDetector.h>
#include <trajectory_planner/polyTrajOccMap.h>
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <trajectory_planner/bsplineTraj.h>

namespace AutoFlight{
	class navigationWaypoint : public flightBase{
	private:
		std::shared_ptr<mapManager::occMap> map_;
		// std::shared_ptr<mapManager::dynamicMap> map_;
		std::shared_ptr<onboardVision::fakeDetector> detector_;
		std::shared_ptr<trajPlanner::polyTrajOccMap> polyTraj_;
		std::shared_ptr<trajPlanner::pwlTraj> pwlTraj_;
		std::shared_ptr<trajPlanner::bsplineTraj> bsplineTraj_;

		ros::Timer plannerTimer_;
		ros::Timer trajExeTimer_;
		ros::Timer visTimer_;
		ros::Timer freeMapTimer_;
		ros::Timer collisionCheckTimer_;
		
		ros::Publisher polyTrajPub_;
		ros::Publisher pwlTrajPub_;
		ros::Publisher bsplineTrajPub_;

		AutoFlight::trajData td_;
		nav_msgs::Path waypoints_;
		nav_msgs::Path polyTrajMsg_;
		nav_msgs::Path pwlTrajMsg_;
		nav_msgs::Path bsplineTrajMsg_;


		int nextWaypointIdx_ = 0;
		double desiredVel_;
		bool trajValid_ = true;
		bool init_ = false;


	public:
		navigationWaypoint(const ros::NodeHandle& nh);
		void initParam();
		void initModules();
		void registerPub();
		void registerCallback();
		void run(const nav_msgs::Path& waypoints);
		void updateWaypoints(const nav_msgs::Path& waypoints);

		void generatePWLTraj(nav_msgs::Path& pwlTraj);
		void getCurrWaypoints(nav_msgs::Path& waypoints);
		void getStartEndConditions(std::vector<Eigen::Vector3d>& startEndCondition);
		void plannerCB(const ros::TimerEvent&);
		void trajExeCB(const ros::TimerEvent&);
		void visCB(const ros::TimerEvent&);
		void freeMapCB(const ros::TimerEvent&);
		void collisionCheckCB(const ros::TimerEvent&);
		void getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclesPos, std::vector<Eigen::Vector3d>& obstaclesVel, std::vector<Eigen::Vector3d>& obstaclesSize);
	};
}

#endif