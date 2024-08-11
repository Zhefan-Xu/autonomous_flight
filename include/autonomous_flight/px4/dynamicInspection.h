/*
	FILE: dynamicInspection.h
	-----------------------------
	header of dynamic inspection
*/
#ifndef DYNAMIC_INSPECTION
#define DYNAMIC_INSPECTION
#include <autonomous_flight/px4/flightBase.h>
#include <map_manager/dynamicMap.h>
#include <onboard_detector/fakeDetector.h>
#include <global_planner/rrtOccMap.h>
#include <trajectory_planner/polyTrajOccMap.h>
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <trajectory_planner/bsplineTraj.h>


namespace AutoFlight{
	
	enum FLIGHT_STATE {FORWARD, EXPLORE, INSPECT, BACKWARD};

	class dynamicInspection : flightBase{
	private:
		ros::NodeHandle nh_;
		ros::Timer plannerTimer_;
		ros::Timer trajExeTimer_;
		ros::Timer checkWallTimer_;
		ros::Timer collisionCheckTimer_;
		ros::Timer replanTimer_;
		ros::Timer visTimer_;
		ros::Timer freeMapTimer_;
		ros::Publisher goalPub_;
		ros::Publisher rrtPathPub_;
		ros::Publisher polyTrajPub_;
		ros::Publisher pwlTrajPub_;
		ros::Publisher bsplineTrajPub_;
		ros::Publisher wallVisPub_;

		// Map
		std::shared_ptr<mapManager::dynamicMap> map_;
		std::shared_ptr<onboardDetector::fakeDetector> detector_;

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
		bool useFakeDetector_;
		double desiredVel_;
		double desiredAcc_;
		double desiredAngularVel_;
		double inspectionVel_; 
		double minWallArea_;
		double safeDistance_;
		double sideSafeDistance_;
		std::vector<double> inspectionHeights_;
		double inspectionHeight_;
		double ascendStep_;
		double descendStep_;
		double sensorRange_;
		double sensorAngleH_;
		double sensorAngleV_;
		int exploreSampleNum_;
		// ***only used when we specify location***
		bool inspectionGoalGiven_ = false;
		std::vector<Eigen::Vector3d> inspectionGoals_;
		Eigen::Vector3d inspectionGoal_;
		std::vector<double> inspectionOrientations_; 
		double inspectionOrientation_;
		bool inspectionWidthGiven_ = false;
		std::vector<double> inspectionWidths_;
		double inspectionWidth_;
		bool zigzagInspection_;
		bool fringeInspection_;
		bool leftFirst_ = true;
		double confirmMaxAngle_;
		bool inspectionConfirm_;
		bool backwardNoTurn_;
		double replanTimeForDynamicObstacle_;
		// ***only used when we specify location***

		// inspection data
		bool trajValid_ = false;
		AutoFlight::trajData td_;
		bool useYaw_ = false;
		std::vector<double> wallRange_;
		bool wallDetected_ = false;
		nav_msgs::Path rrtPathMsg_;
		nav_msgs::Path polyTrajMsg_;
		nav_msgs::Path pwlTrajMsg_;
		nav_msgs::Path bsplineTrajMsg_;
		visualization_msgs::MarkerArray wallVisMsg_;
		bool trajectoryReady_ = false;
		bool replan_ = true;
		ros::Time trajStartTime_;
		double trajTime_; // current trajectory time
		trajPlanner::bspline trajectory_; // trajectory data for navigation
		int countBsplineFailure_ = 0;
		ros::Time lastDynamicObstacleTime_;

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
		void collisionCheckCB(const ros::TimerEvent&); // online collision checking
		void replanCB(const ros::TimerEvent&); // replan callback
		void visCB(const ros::TimerEvent&);
		void freeMapCB(const ros::TimerEvent&);

		geometry_msgs::PoseStamped getForwardGoal();
		nav_msgs::Path getRestGlobalPath();
		void getStartEndConditions(std::vector<Eigen::Vector3d>& startEndCondition);
		void getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclesPos, std::vector<Eigen::Vector3d>& obstaclesVel, std::vector<Eigen::Vector3d>& obstaclesSize);
		void changeState(const FLIGHT_STATE& flightState);


		// basic operations
		bool moveToPosition(const geometry_msgs::Point& position);
		bool moveToPosition(const geometry_msgs::Point& position, double vel);
		bool moveToPosition(const Eigen::Vector3d& position);
		bool moveToPosition(const Eigen::Vector3d& position, double vel);
		bool moveToOrientation(const geometry_msgs::Quaternion& orientation);
		bool moveToOrientation(double yaw);
		bool moveToOrientationStep(double yaw);
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
		visualization_msgs::Marker getLineMarker(double x1, double y1, double z1, double x2, double y2, double z2, int id, bool isWall);
		void getWallVisMsg(visualization_msgs::MarkerArray& msg);


		// inspection module
		void checkSurroundings();
		void inspectZigZag();
		void inspectZigZagRange();
		void inspectFringe();
		void inspectFringeRange();

		// navigation
		bool hasCollision();
		bool hasDynamicCollision();
		double computeExecutionDistance();
		bool replanForDynamicObstacle();
		nav_msgs::Path getCurrentTraj(double dt);
		
		// utils
		geometry_msgs::PoseStamped eigen2ps(const Eigen::Vector3d& p);
	};
}

#endif
