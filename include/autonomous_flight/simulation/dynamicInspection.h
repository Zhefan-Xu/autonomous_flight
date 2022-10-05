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
	
	enum FLIGHT_STATE {FORWARD, EXPLORE, INSPECT, BACK, STOP};

	class dynamicInspection : flightBase{
	private:
		ros::NodeHandle nh_;
		ros::Timer plannerTimer_;
		ros::Timer trajExeTimer_;
		ros::Timer checkWallTimer_;
		ros::Timer visTimer_;
		ros::Publisher goalPub_;
		ros::Publisher pwlTrajPub_;
		ros::Publisher bsplineTrajPub_;
		ros::Publisher wallVisPub_;

		// Map
		std::shared_ptr<mapManager::occMap> map_;

		// Planner
		std::shared_ptr<globalPlanner::rrtOccMap<3>> rrtPlanner_;
		std::shared_ptr<trajPlanner::polyTrajOccMap> polyTraj_;
		std::shared_ptr<trajPlanner::pwlTraj> pwlTraj_;
		std::shared_ptr<trajPlanner::bsplineTraj> bsplineTraj_;


		// state
		FLIGHT_STATE flightState_ = FLIGHT_STATE::FORWARD;
		geometry_msgs::PoseStamped goal_;

		// inspection parameters
		double desiredVel_;
		double minWallArea_;


		// inspection data
		AutoFlight::trajData td_;
		std::vector<double> wallRange_;
		nav_msgs::Path pwlTrajMsg_;
		nav_msgs::Path bsplineTrajMsg_;
		visualization_msgs::MarkerArray wallVisMsg_;

	public:
		dynamicInspection();
		dynamicInspection(const ros::NodeHandle& nh);
		void initParam();
		void initModules();
		void registerPub();
		void registerCallback();
		
		void run();

		void plannerCB(const ros::TimerEvent&);
		void trajExeCB(const ros::TimerEvent&);
		void checkWallCB(const ros::TimerEvent&); // check whether the front wall is reached
		void visCB(const ros::TimerEvent&);

		geometry_msgs::PoseStamped getForwardGoal();
		void getStartEndConditions(std::vector<Eigen::Vector3d>& startEndCondition);
		void changeState(const FLIGHT_STATE& flightState);


		// wall detection
		bool isWallDetected();
		void updateWallRange(const std::vector<double>& wallRange);
		visualization_msgs::Marker getLineMarker(double x1, double y1, double z1, double x2, double y2, double z2, int id, bool isWall);
		void getWallVisMsg(visualization_msgs::MarkerArray& msg);
	};
}

#endif