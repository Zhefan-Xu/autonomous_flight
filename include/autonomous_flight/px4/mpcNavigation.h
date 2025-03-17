/*
	FILE: mpcNavigation.h
	------------------------
	dynamic navigation header file
*/

#ifndef AUTOFLIGHT_MPC_NAVIGATION_H
#define AUTOFLIGHT_MPC_NAVIGATION_H

#include <ros/package.h>
#include <autonomous_flight/px4/flightBase.h>
#include <map_manager/dynamicMap.h>
#include <onboard_detector/fakeDetector.h>
#include <dynamic_predictor/dynamicPredictor.h>
#include <global_planner/rrtOccMap.h>
#include <trajectory_planner/polyTrajOccMap.h>
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <trajectory_planner/bsplineTraj.h>
#include <trajectory_planner/mpcPlanner.h>

namespace AutoFlight{

	class mpcNavigation : public flightBase{
	private:
		std::shared_ptr<mapManager::dynamicMap> map_;
		std::shared_ptr<onboardDetector::fakeDetector> detector_;
		std::shared_ptr<dynamicPredictor::predictor> predictor_;
		std::shared_ptr<globalPlanner::rrtOccMap<3>> rrtPlanner_;
		std::shared_ptr<trajPlanner::polyTrajOccMap> polyTraj_;
		std::shared_ptr<trajPlanner::pwlTraj> pwlTraj_;
		std::shared_ptr<trajPlanner::mpcPlanner> mpc_;

		ros::Timer replanCheckTimer_;
		ros::Timer trajExeTimer_;
		ros::Timer visTimer_;
		ros::Timer freeMapTimer_;

		ros::Publisher rrtPathPub_;
		ros::Publisher polyTrajPub_;
		ros::Publisher pwlTrajPub_;
		ros::Publisher mpcTrajPub_;
		ros::Publisher inputTrajPub_;
		ros::Publisher goalPub_;

		std::thread mpcWorker_;

		// parameters
		bool useFakeDetector_;
		bool usePredictor_;
		bool useGlobalPlanner_;
		bool noYawTurning_;
		bool useYawControl_;
		bool usePredefinedGoal_;
		double desiredVel_;
		double desiredAcc_;
		double desiredAngularVel_;
		std::string refTrajPath_;
		nav_msgs::Path predefinedGoal_;
		int goalIdx_ = 0;
		int repeatPathNum_;

		// navigation data
		bool mpcReplan_ = false;
		bool replanning_ = false;
		bool needGlobalPlan_ = false;
		bool globalPlanReady_ = false;
		bool refTrajReady_ = false;
		bool mpcFirstTime_ = false;
		nav_msgs::Path rrtPathMsg_;
		nav_msgs::Path polyTrajMsg_;
		nav_msgs::Path pwlTrajMsg_;
		nav_msgs::Path mpcTrajMsg_;
		nav_msgs::Path inputTrajMsg_;
		bool mpcTrajectoryReady_ = false;
		ros::Time trajStartTime_;
		ros::Time trackingStartTime_;
		double trajTime_; // current trajectory time
		double prevInputTrajTime_ = 0.0;
		trajPlanner::bspline trajectory_; // trajectory data for tracking
		double facingYaw_;
		bool firstTimeSave_ = false;
		bool lastDynamicObstacle_ = false;
		ros::Time lastDynamicObstacleTime_;
		
	public:
		mpcNavigation(const ros::NodeHandle& nh);
		void initParam();
		void initModules();
		void registerPub();
		void registerCallback();

		void mpcCB();
		void staticPlannerCB(const ros::TimerEvent&);
		void replanCheckCB(const ros::TimerEvent&);
		void trajExeCB(const ros::TimerEvent&);
		void visCB(const ros::TimerEvent&);
		void freeMapCB(const ros::TimerEvent&); // using fake detector

		void run();	
		bool goalHasCollision();
		bool mpcHasCollision();
		bool hasCollision();
		bool hasDynamicCollision();
		nav_msgs::Path getRefTraj();
		nav_msgs::Path getCurrentTraj(double dt);
		nav_msgs::Path getRestGlobalPath();
		void getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclesPos, std::vector<Eigen::Vector3d>& obstaclesVel, std::vector<Eigen::Vector3d>& obstaclesSize, const Eigen::Vector3d &robotSize = Eigen::Vector3d(0.0, 0.0, 0.0));
		void publishGoal();
	};
}

#endif