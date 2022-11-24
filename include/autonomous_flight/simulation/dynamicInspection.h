/*
	FILE: dynamicInspection.h
	-----------------------------
	header of dynamic inspection
*/
#ifndef DYNAMIC_INSPECTION
#define DYNAMIC_INSPECTION
#include <autonomous_flight/simulation/flightBase.h>
#include <map_manager/dynamicMap.h>
#include <global_planner/rrtOccMap.h>
#include <trajectory_planner/polyTrajOccMap.h>
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <trajectory_planner/bsplineTraj.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


namespace AutoFlight{
	
	enum FLIGHT_STATE {FORWARD, EXPLORE, INSPECT, BACKWARD};

	class dynamicInspection : flightBase{
	private:
		ros::NodeHandle nh_;
		ros::Timer plannerTimer_;
		ros::Timer trajExeTimer_;
		ros::Timer checkWallTimer_;
		ros::Timer sideWallRaycastTimer_;
		ros::Timer collisionCheckTimer_;
		ros::Timer visTimer_;
		ros::Publisher goalPub_;
		ros::Publisher rrtPathPub_;
		ros::Publisher polyTrajPub_;
		ros::Publisher pwlTrajPub_;
		ros::Publisher bsplineTrajPub_;
		ros::Publisher wallVisPub_;
		image_transport::Publisher sideWallRaycastVisPub_;

		// Map
		std::shared_ptr<mapManager::dynamicMap> map_;

		// Planner
		std::shared_ptr<globalPlanner::rrtOccMap<3>> rrtPlanner_;
		std::shared_ptr<trajPlanner::polyTrajOccMap> polyTraj_;
		std::shared_ptr<trajPlanner::pwlTraj> pwlTraj_;
		std::shared_ptr<trajPlanner::bsplineTraj> bsplineTraj_;


		// state
		FLIGHT_STATE flightState_ = FLIGHT_STATE::FORWARD;
		FLIGHT_STATE prevState_ = FLIGHT_STATE::FORWARD;
		geometry_msgs::PoseStamped goal_;

		// inspection parameters
		double desiredVel_;
		double desiredAngularVel_;
		double inspectionVel_; 
		double minWallArea_;
		double safeDistance_;
		double sideSafeDistance_;
		double inspectionHeight_;
		double ascendStep_;
		double descendStep_;
		double sensorRange_;
		double sensorAngleH_;
		double sensorAngleV_;
		int exploreSampleNum_;
		// ***only used when we specify location***
		bool inspectionGoalGiven_ = false;
		Eigen::Vector3d inspectionGoal_; 
		double inspectionOrientation_;
		bool inspectionWidthGiven_ = false;
		double inspectionWidth_;
		double sideWallRaycastRange_;
		double sideWallLengthThresh_;
		double sideWallAngleThresh_;
		// ***only used when we specify location***

		// inspection data
		bool trajValid_ = false;
		AutoFlight::trajData td_;
		bool useYaw_ = false;
		std::vector<double> wallRange_;
		std::vector<Eigen::Vector3d> frontWallPoints_;
		Eigen::Vector3d frontWallDim_;
		nav_msgs::Path rrtPathMsg_;
		nav_msgs::Path polyTrajMsg_;
		nav_msgs::Path pwlTrajMsg_;
		nav_msgs::Path bsplineTrajMsg_;
		visualization_msgs::MarkerArray wallVisMsg_;
		int countBsplineFailure_ = 0;
		std::vector<Eigen::Vector3d> sideWallPoints_;
		std::vector<Eigen::Vector3d> sideWallPointsC_;
		std::vector<Eigen::Vector3d> sideWallPointsAll_; // in all directions
		std::vector<Eigen::Vector3d> finalSideWallPoints_;
		std::vector<Eigen::Vector3d> finalSideWallPointsC_;
		sensor_msgs::ImagePtr sideWallRaycastImg_;
		double movingDirection_ = -12345.0;

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
		void sideWallRaycastCB(const ros::TimerEvent&);
		void collisionCheckCB(const ros::TimerEvent&); // online collision checking
		void visCB(const ros::TimerEvent&);

		geometry_msgs::PoseStamped getForwardGoal();
		nav_msgs::Path getLatestGlobalPath();
		void getStartEndConditions(std::vector<Eigen::Vector3d>& startEndCondition);
		void changeState(const FLIGHT_STATE& flightState);


		// basic operations
		bool moveToPosition(const geometry_msgs::Point& position);
		bool moveToPosition(const geometry_msgs::Point& position, double vel);
		bool moveToPosition(const Eigen::Vector3d& position);
		bool moveToPosition(const Eigen::Vector3d& position, double vel);
		bool moveToOrientation(const geometry_msgs::Quaternion& orientation);
		bool moveToOrientation(double yaw);
		double makePWLTraj(const std::vector<geometry_msgs::PoseStamped>& waypoints, nav_msgs::Path& resultPath);
		double makePWLTraj(const std::vector<geometry_msgs::PoseStamped>& waypoints, double desiredVel, nav_msgs::Path& resultPath);
		double getPathLength(const nav_msgs::Path& path);

		// exploration module
		bool getBestViewPoint(Eigen::Vector3d& bestPoint);
		bool randomSample(Eigen::Vector3d& pSample);
		bool satisfyWallDistance(const Eigen::Vector3d& p);
		int countUnknownFOV(const Eigen::Vector3d& p, double yaw);
		void setStartPositionFree();


		// wall detection module
		bool castRayOccupied(const Eigen::Vector3d& start, const Eigen::Vector3d& direction, Eigen::Vector3d& end, double maxRayLength);
		bool isWallDetected();
		double getWallDistance();
		void updateWallRange(const std::vector<double>& wallRange);
		void updateFrontWallPoints(const std::vector<Eigen::Vector3d>& frontWallPoints);
		void updateFrontWallDim(double length, double width, double height);
		visualization_msgs::Marker getLineMarker(double x1, double y1, double z1, double x2, double y2, double z2, int id, bool isWall);
		visualization_msgs::Marker getLineMarker(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, int id, bool isWall);
		void getWallVisMsg(visualization_msgs::MarkerArray& msg);


		// inspection module
		void checkSurroundings();
		void inspectZigZag();
		void inspectZigZagRange();
		void inspectFringe();
		void inspectFringeRange();


		// side wall raycasting module
		double getMovingDirection();
		void sideCast(int rayNum, std::vector<Eigen::Vector3d>& rayEndVec, std::vector<int>& rayIDVec);
		void getPotentialWallPoints(int rayNum, const std::vector<Eigen::Vector3d>& rayEndVec, const std::vector<int>& rayIDVec, std::vector<std::vector<Eigen::Vector3d>>& potentialWallVec);
		void checkPotentialWallLength(std::vector<std::vector<Eigen::Vector3d>>& potentialWallVec);
		double calculateWallLength(const std::vector<Eigen::Vector3d>& points);
		void checkPotentialWallAngle(std::vector<std::vector<Eigen::Vector3d>>& potentialWallVec);
		double checkWallPointMinAngle(const std::vector<Eigen::Vector3d>& points);
		void getSideWallPoints(const std::vector<std::vector<Eigen::Vector3d>>& potentialWallVec, std::vector<Eigen::Vector3d>& finalSideWallPoints);
		void getSideWallRaycastMsg(sensor_msgs::ImagePtr& imgMsg);

		
		// utils
		geometry_msgs::PoseStamped eigen2ps(const Eigen::Vector3d& p);
	};
}

#endif
