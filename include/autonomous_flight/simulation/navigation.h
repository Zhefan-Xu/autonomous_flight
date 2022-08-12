/*
	FILE: navigation.h
	------------------------
	navigation header file in simulation
*/

#ifndef AUTOFLIGHT_NAVIGATION_H
#define AUTOFLIGHT_NAVIGATION_H

#include <autonomous_flight/simulation/flightBase.h>
#include <map_manager/occupancyMap.h>
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <trajectory_planner/bsplineTraj.h>

namespace AutoFlight{
	class navigation : public flightBase{
	private:
		std::shared_ptr<mapManager::occMap> map_;
		std::shared_ptr<trajPlanner::pwlTraj> pwlTraj_;
		std::shared_ptr<trajPlanner::bsplineTraj> bsplineTraj_;

		ros::Timer pwlTimer_;
		ros::Timer bsplineTimer_;
		ros::Timer trajExeTimer_;
		ros::Publisher pwlTrajPub_;
		ros::Publisher bsplineTrajPub_;

		AutoFlight::trajData td_;
		nav_msgs::Path pwlTrajMsg_;
		nav_msgs::Path bsplineTrajMsg_;

		double desiredVel_;

		int restartNum_ = 0;
		bool pwlTrajUpdated_ = false;
		bool needNewPwlTraj_ = false;
		bool stopBsplinePlan_ = false;

	public:
		navigation(const ros::NodeHandle& nh);
		void initParam();
		void initModules();
		void registerPub();
		void registerCallback();
		void run();

		void pwlCB(const ros::TimerEvent&);
		void bsplineCB(const ros::TimerEvent&);
		void trajExeCB(const ros::TimerEvent&);
	};
}

#endif