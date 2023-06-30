/*
	FILE: dynamicExploration.h
	-----------------------------
	header of dynamic exploration
*/
#ifndef DYNAMIC_EXPLORATION
#define DYNAMIC_EXPLORATION
#include <autonomous_flight/px4/flightBase.h>
#include <global_planner/dep.h>
#include <trajectory_planner/polyTrajOccMap.h>
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <trajectory_planner/bsplineTraj.h>
#include <map_manager/dynamicMap.h>

namespace AutoFlight{
	class dynamicExploration : flightBase{
	private:
		std::shared_ptr<mapManager::dynamicMap> map_;
		std::shared_ptr<globalPlanner::DEP> expPlanner_;
		std::shared_ptr<trajPlanner::polyTrajOccMap> polyTraj_;
		std::shared_ptr<trajPlanner::pwlTraj> pwlTraj_;
		std::shared_ptr<trajPlanner::bsplineTraj> bsplineTraj_;

		ros::Timer plannerTimer_;
		ros::Timer replanCheckTimer_;
		ros::Timer trajExeTimer_;
		ros::Timer visTimer_;

		ros::Publisher inputTrajPub_;

		// parameters
		double desiredVel_;
		double desiredAcc_;
		double desiredAngularVel_;

		// exploration data
		bool replan_ = false;
		bool newWaypoints_ = false;
		nav_msgs::Path waypoints_; // latest waypoints from exploration planner
		nav_msgs::Path inputTrajMsg_;
		nav_msgs::Path polyTrajMsg_;
		nav_msgs::Path pwlTrajMsg_;
		nav_msgs::Path bsplineTrajMsg_;
	
	public:
		std::thread exploreReplanWorker_;
		dynamicExploration();
		dynamicExploration(const ros::NodeHandle& nh);

		void initParam();
		void initModules();
		void registerCallback();
		void registerPub();

		void plannerCB(const ros::TimerEvent&);
		void replanCheckCB(const ros::TimerEvent&);
		void trajExeCB(const ros::TimerEvent&);
		void visCB(const ros::TimerEvent&);

		void run();
		void getStartEndConditions(std::vector<Eigen::Vector3d>& startEndConditions);
		void exploreReplan();
	};
}
#endif