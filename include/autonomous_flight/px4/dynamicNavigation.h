/*
	FILE: dynamicNavigation.h
	------------------------
	dynamic navigation header file in real world
*/

#ifndef AUTOFLIGHT_DYNAMIC_NAVIGATION_H
#define AUTOFLIGHT_DYNAMIC_NAVIGATION_H

#include <autonomous_flight/px4/flightBase.h>
#include <map_manager/dynamicMap.h>
#include <onboard_vision/fakeDetector.h>
#include <global_planner/rrtOccMap.h>
#include <trajectory_planner/polyTrajOccMap.h>
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <trajectory_planner/bsplineTraj.h>

namespace AutoFlight{
	class dynamicNavigation : public flightBase{
	private:
		std::shared_ptr<mapManager::dynamicMap> map_;
		std::shared_ptr<trajPlanner::pwlTraj> pwlTraj_;
		std::shared_ptr<trajPlanner::bsplineTraj> bsplineTraj_;

		ros::Timer plannerTimer_;
		ros::Timer trajExeTimer_;
		ros::Timer historyTrajTimer_;
		ros::Timer visTimer_;
		ros::Timer collisionCheckTimer_;

		ros::Publisher pwlTrajPub_;
		ros::Publisher bsplineTrajPub_;
		ros::Publisher historyTrajPub_;

		AutoFlight::trajData td_;
		nav_msgs::Path pwlTrajMsg_;
		nav_msgs::Path bsplineTrajMsg_;
		nav_msgs::Path historyTrajMsg_;


		double desiredVel_;
		bool trajValid_ = true;
		bool adjustHeading_ = false;


	public:
		dynamicNavigation(const ros::NodeHandle& nh);
		void initParam();
		void initModules();
		void registerPub();
		void registerCallback();
		void run();
		void adjustHeading();
		void getStartCondition(std::vector<Eigen::Vector3d>& startEndCondition);

		void plannerCB(const ros::TimerEvent&);
		void trajExeCB(const ros::TimerEvent&);
		void historyTrajCB(const ros::TimerEvent&);
		void visCB(const ros::TimerEvent&);
		void collisionCheckCB(const ros::TimerEvent&);
	};
}

#endif