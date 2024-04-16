/*
	FILE: navigation.h
	------------------------
	navigation header file in px4
*/

#ifndef AUTOFLIGHT_NAVIGATION_H
#define AUTOFLIGHT_NAVIGATION_H

#include <autonomous_flight/px4/flightBase.h>
#include <map_manager/occupancyMap.h>
#include <global_planner/rrtOccMap.h>
#include <trajectory_planner/polyTrajOccMap.h>
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <trajectory_planner/bsplineTraj.h>
#include <trajectory_planner/mpcPlanner.h>
#include <time_optimizer/trajectoryDivider.h>
#include <time_optimizer/bsplineTimeOptimizer.h>

namespace AutoFlight{
	class navigation : public flightBase{
	private:
		std::shared_ptr<mapManager::occMap> map_;
		std::shared_ptr<globalPlanner::rrtOccMap<3>> rrtPlanner_;
		std::shared_ptr<trajPlanner::polyTrajOccMap> polyTraj_;
		std::shared_ptr<trajPlanner::pwlTraj> pwlTraj_;
		std::shared_ptr<trajPlanner::bsplineTraj> bsplineTraj_;
		std::shared_ptr<timeOptimizer::trajDivider> trajDivider_;
		std::shared_ptr<timeOptimizer::bsplineTimeOptimizer> timeOptimizer_;
		std::shared_ptr<trajPlanner::mpcPlanner> mpc_;

		ros::Timer mpcTimer_;
		ros::Timer plannerTimer_;
		ros::Timer replanCheckTimer_;
		ros::Timer trajExeTimer_;
		ros::Timer visTimer_;

		ros::Publisher rrtPathPub_;
		ros::Publisher polyTrajPub_;
		ros::Publisher pwlTrajPub_;
		ros::Publisher bsplineTrajPub_;
		ros::Publisher mpcTrajPub_;
		ros::Publisher inputTrajPub_;
		ros::Publisher inputTrajPointsPub_;
		ros::Publisher goalPub_;

		std::thread trajExeWorker_;
		std::thread mpcWorker_;
		// parameters
		bool useGlobalPlanner_;
		bool useMPCPlanner_;
		bool noYawTurning_;
		bool useYawControl_;
		bool usePredefinedGoal_;
		double desiredVel_;
		double desiredAcc_;
		double desiredAngularVel_;
		std::string trajSavePath_;
		bool useTimeOptimizer_;
		nav_msgs::Path predefinedGoal_;
		int goalIdx_ = 0;
		int repeatPathNum_;
		// navigation data
		bool replan_ = false;
		bool replanning_ = false;
		bool needGlobalPlan_ = false;
		bool globalPlanReady_ = false;
		bool refTrajReady_ = false;
		bool mpcFirstTime_ = false;
		nav_msgs::Path rrtPathMsg_;
		nav_msgs::Path polyTrajMsg_;
		nav_msgs::Path pwlTrajMsg_;
		nav_msgs::Path bsplineTrajMsg_;
		nav_msgs::Path mpcTrajMsg_;
		nav_msgs::Path inputTrajMsg_;
		bool trajectoryReady_ = false;
		ros::Time trajStartTime_;
		double trajTime_; // current trajectory time
		double prevInputTrajTime_ = 0.0;
		double facingYaw_;
		trajPlanner::bspline trajectory_; // trajectory data for tracking
		bool firstTimeSave_ = false;
		


	public:
		navigation(const ros::NodeHandle& nh);
		void initParam();
		void initModules();
		void registerPub();
		void registerCallback();

		void mpcCB();
		void plannerCB(const ros::TimerEvent&);
		void replanCheckCB(const ros::TimerEvent&);
		void trajExeCB(const ros::TimerEvent&);
		void visCB(const ros::TimerEvent&);

		void run();	
		void getStartEndConditions(std::vector<Eigen::Vector3d>& startEndConditions);	
		bool goalHasCollision();
		bool hasCollision();
		double computeExecutionDistance();
		nav_msgs::Path getCurrentTraj(double dt);
		nav_msgs::Path getRestGlobalPath();
		void publishInputTraj();
		void publishGoal();
	};
}

#endif