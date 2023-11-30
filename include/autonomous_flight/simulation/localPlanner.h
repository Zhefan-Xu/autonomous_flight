/*
	FILE: localPlanner.h
	---------------------------------------------
	local planner header file
*/
#ifndef AUTONOMOUSFLIGHT_LOCAL_PLANNER
#define AUTONOMOUSFLIGHT_LOCAL_PLANNER

#include <map_manager/dynamicMap.h>
#include <trajectory_planner/polyTrajOccMap.h>
#include <trajectory_planner/bsplineTraj.h>
#include <time_optimizer/bsplineTimeOptimizer.h>

namespace AutoFlight{
	class localPlanner{
	private:
		ros::NodeHandle nh_;
		std::shared_ptr<mapManager::occMap> occMap_;
		std::shared_ptr<mapManager::dynamicMap> dynamicMap_;

		// planners
		std::shared_ptr<trajPlanner::polyTrajOccMap> polyTraj_;
		std::shared_ptr<trajPlanner::pwlTraj> pwlTraj_;
		std::shared_ptr<trajPlanner::bsplineTraj> bsplineTraj_;
		std::shared_ptr<timeOptimizer::bsplineTimeOptimizer> timeOptimizer_;


		double desiredVel_;
		double desiredAcc_;

	public:
		localPlanner(const ros::NodeHandle& nh, double desiredVel, double desiredAcc);
		void loadOccMap(const std::shared_ptr<mapManager::occMap> occMap);
		void loadDynamicMap(const std::shared_ptr<mapManager::dynamicMap> dynamicMap);

	};
};


#endif
