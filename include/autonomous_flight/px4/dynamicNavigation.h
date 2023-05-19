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
		std::shared_ptr<onboardVision::fakeDetector> detector_;
		std::shared_ptr<globalPlanner::rrtOccMap<3>> rrtPlanner_;
		std::shared_ptr<trajPlanner::polyTrajOccMap> polyTraj_;
		std::shared_ptr<trajPlanner::pwlTraj> pwlTraj_;
		std::shared_ptr<trajPlanner::bsplineTraj> bsplineTraj_;

		ros::Timer plannerTimer_;
		ros::Timer replanCheckTimer_;
		ros::Timer trajExeTimer_;
		ros::Timer stateUpdateTimer_;
		ros::Timer visTimer_;
		ros::Timer freeMapTimer_;

		ros::Publisher rrtPathPub_;
		ros::Publisher polyTrajPub_;
		ros::Publisher pwlTrajPub_;
		ros::Publisher bsplineTrajPub_;
		ros::Publisher inputTrajPub_;

		// parameters
		bool useFakeDetector_;
		bool useGlobalPlanner_;
		bool noYawTurning_;
		bool useYawControl_;
		double desiredVel_;
		double desiredAcc_;
		double desiredAngularVel_;
		std::string trajSavePath_;

		// navigation data
		bool stateUpdateFirstTime_ = true;
		ros::Time prevStateTime_;
		Eigen::Vector3d currVel_, currAcc_, prevVel_;
		bool replan_ = false;
		bool needGlobalPlan_ = false;
		bool globalPlanReady_ = false;
		int nextWaypointIdx_ = 0;
		nav_msgs::Path rrtPathMsg_;
		nav_msgs::Path polyTrajMsg_;
		nav_msgs::Path pwlTrajMsg_;
		nav_msgs::Path bsplineTrajMsg_;
		nav_msgs::Path inputTrajMsg_;
		bool trajectoryReady_ = false;
		ros::Time trajStartTime_;
		double trajTime_; // current trajectory time
		double prevInputTrajTime_ = 0.0;
		trajPlanner::bspline trajectory_; // trajectory data for tracking
		bool firstTimeSave_ = false;
		



	public:
		dynamicNavigation(const ros::NodeHandle& nh);
		void initParam();
		void initModules();
		void registerPub();
		void registerCallback();

		void plannerCB(const ros::TimerEvent&);
		void replanCheckCB(const ros::TimerEvent&);
		void trajExeCB(const ros::TimerEvent&);
		void stateUpdateCB(const ros::TimerEvent&);
		void visCB(const ros::TimerEvent&);
		void freeMapCB(const ros::TimerEvent&); // using fake detector

		void run();	
		void getStartEndCondition(std::vector<Eigen::Vector3d>& startEndCondition);	
		bool hasCollision();
		bool hasDynamicCollision();
		double computeExecutionDistance();
		nav_msgs::Path getCurrentTraj(double dt);
		nav_msgs::Path getRestGlobalPath();
		void getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclesPos, std::vector<Eigen::Vector3d>& obstaclesVel, std::vector<Eigen::Vector3d>& obstaclesSize);
	};
}

#endif