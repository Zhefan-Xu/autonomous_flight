/*
	FILE: testNavigaton.h
	------------------------
	test navigation header file in simulation
*/

#ifndef AUTOFLIGHT_TEST_NAVIGATION_H
#define AUTOFLIGHT_TEST_NAVIGATION_H

#include <autonomous_flight/simulation/flightBase.h>
#include <map_manager/occupancyMap.h>
#include <onboard_vision/fakeDetector.h>
#include <global_planner/rrtOccMap.h>
#include <trajectory_planner/polyTrajOccMap.h>
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <trajectory_planner/bsplineTraj.h>

namespace AutoFlight{
	class testNavigation : public flightBase{
	private:
		std::shared_ptr<mapManager::occMap> map_;
		std::shared_ptr<onboardVision::fakeDetector> detector_;
		std::shared_ptr<trajPlanner::pwlTraj> pwlTraj_;
		std::shared_ptr<trajPlanner::bsplineTraj> bsplineTraj_;

		ros::Timer plannerTimer_;
		ros::Timer trajExeTimer_;
		ros::Timer visTimer_;
		ros::Timer freeMapTimer_;
		ros::Timer collisionCheckTimer_;

		ros::Publisher pwlTrajPub_;
		ros::Publisher bsplineTrajPub_;

		AutoFlight::trajData td_;
		nav_msgs::Path pwlTrajMsg_;
		nav_msgs::Path bsplineTrajMsg_;


		double desiredVel_;
		bool trajValid_ = true;

	public:
		testNavigation(const ros::NodeHandle& nh);
		void initParam();
		void initModules();
		void registerPub();
		void registerCallback();
		void run();

		void plannerCB(const ros::TimerEvent&);
		void trajExeCB(const ros::TimerEvent&);
		void visCB(const ros::TimerEvent&);
		void freeMapCB(const ros::TimerEvent&);
		void collisionCheckCB(const ros::TimerEvent&);

		// helper function
		void getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclesPos, std::vector<Eigen::Vector3d>& obstaclesVel, std::vector<Eigen::Vector3d>& obstaclesSize);
	};
}

#endif