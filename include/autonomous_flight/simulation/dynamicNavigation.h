/*
	FILE: dynamicNavigation.h
	------------------------
	dynamic navigation header file in simulation
*/

#ifndef AUTOFLIGHT_DYNAMIC_NAVIGATION_H
#define AUTOFLIGHT_DYNAMIC_NAVIGATION_H

#include <autonomous_flight/simulation/flightBase.h>
#include <map_manager/occupancyMap.h>
#include <onboard_vision/fakeDetector.h>
#include <global_planner/rrtOccMap.h>
#include <trajectory_planner/polyTrajOccMap.h>
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <trajectory_planner/bsplineTraj.h>

namespace AutoFlight{
	class dynamicNavigation : public flightBase{
	private:
		std::shared_ptr<mapManager::occMap> map_;
		std::shared_ptr<onboardVision::fakeDetector> detector_;
		std::shared_ptr<globalPlanner::rrtOccMap<3>> rrtPlanner_;
		std::shared_ptr<trajPlanner::polyTrajOccMap> polyTraj_;
		std::shared_ptr<trajPlanner::pwlTraj> pwlTraj_;
		std::shared_ptr<trajPlanner::bsplineTraj> bsplineTraj_;

		ros::Timer rrtTimer_;
		ros::Timer polyTrajTimer_;
		ros::Timer pwlTimer_;
		ros::Timer bsplineTimer_;
		ros::Timer trajExeTimer_;
		ros::Timer visTimer_;
		ros::Timer freeMapTimer_;
		
		ros::Publisher rrtPathPub_;
		ros::Publisher polyTrajPub_;
		ros::Publisher pwlTrajPub_;
		ros::Publisher bsplineTrajPub_;

		AutoFlight::trajData td_;
		nav_msgs::Path rrtPathMsg_;
		nav_msgs::Path polyTrajMsg_;
		nav_msgs::Path pwlTrajMsg_;
		nav_msgs::Path bsplineTrajMsg_;


		double desiredVel_;
		bool goalReceivedPWL_ = false;
		bool rrtPathUpdated_ = false;
		bool bsplineFailure_ = false;
		bool useGlobalTraj_ = false;


	public:
		dynamicNavigation(const ros::NodeHandle& nh);
		void initParam();
		void initModules();
		void registerPub();
		void registerCallback();
		void run();

		void rrtCB(const ros::TimerEvent&);
		void polyTrajCB(const ros::TimerEvent&);
		void pwlCB(const ros::TimerEvent&);
		void bsplineCB(const ros::TimerEvent&);
		void trajExeCB(const ros::TimerEvent&);
		void visCB(const ros::TimerEvent&);
		void freeMapCB(const ros::TimerEvent&);

		// helper function
		void getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclesPos, std::vector<Eigen::Vector3d>& obstaclesVel, std::vector<Eigen::Vector3d>& obstaclesSize);
	};
}

#endif