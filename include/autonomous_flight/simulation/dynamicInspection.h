/*
	FILE: dynamicInspection.h
	-----------------------------
	header of dynamic inspection
*/
#ifndef DYNAMIC_INSPECTION
#define DYNAMIC_INSPECTION
#include <autonomous_flight/simulation/flightBase.h>
#include <map_manager/occupancyMap.h>
#include <global_planner/rrtOccMap.h>
#include <trajectory_planner/polyTrajOccMap.h>
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <trajectory_planner/bsplineTraj.h>

namespace AutoFlight{
	
	enum FLIGHT_STATE {NAVIGATE, EXPLORE, INSPECT, BACK, STOP};

	class dynamicInspection : flightBase{
	private:
		ros::NodeHandle nh_;
		ros::Timer plannerTimer_;

		// Map
		std::shared_ptr<mapManager::occMap> map_;

		// Planner
		std::shared_ptr<globalPlanner::rrtOccMap<3>> rrtPlanner_;
		std::shared_ptr<trajPlanner::polyTrajOccMap> polyTraj_;
		std::shared_ptr<trajPlanner::pwlTraj> pwlTraj_;
		std::shared_ptr<trajPlanner::bsplineTraj> bsplineTraj_;


		// state
		FLIGHT_STATE flightState_ = FLIGHT_STATE::NAVIGATE;
		geometry_msgs::PoseStamped goal_;

	public:
		dynamicInspection();
		dynamicInspection(const ros::NodeHandle& nh);
		void registerCallback();
		void run();
		void plannerCB(const ros::TimerEvent&);
		void changeGoal(double x, double y, double z);
		void changeGoal(const geometry_msgs::PoseStamped& goal);
		void changeState(const FLIGHT_STATE& flightState);
	};
}

#endif