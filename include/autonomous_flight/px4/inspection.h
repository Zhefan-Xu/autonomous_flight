/*
	FILE: inspection.h
	---------------------------
	autonomous inspection header file
*/
#ifndef INSPECTION_H
#define INSPECTION_H
#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/Path.h>
#include <global_planner/rrtStarOctomap.h>
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <autonomous_flight/px4/flightBase.h>
#include <algorithm>
#include <math.h>
#include <visualization_msgs/MarkerArray.h>

using std::cout; using std::endl;
namespace AutoFlight{
	class inspector : public flightBase{
	private:
		ros::Subscriber mapSub_;
		ros::Publisher targetVisPub_; // visualization
		ros::Publisher pathPub_; // visualization
		ros::Publisher avoidancePathVisPub_; // visualization

		// parameters
		double sampleTime_ = 0.1;
		std::vector<double> collisionBox_;
		double frontSafeDist_;
		double avoidSafeDist_;
		double sideSafeDist_;
		double zigZagSafeDist_;
		double avoidanceOnlineCheck_;

		double minTargetArea_; // min area to be considered as the target
		double maxTargetHgt_; // max range of inspection target height
		double maxTargetWidth_; // max range of inspection target width
		double descendHgt_;
		double desiredVel_;
		double desiredAngularVel_;
		int nbvSampleNum_;
		double sensorRange_;
		double sensorVerticalAngle_;
		double forwardMinDist_;
		double stepAscendDelta_;
		double lookAroundAngle_;
		double checkTargetLookAroundAngle_;
		std::vector<double> startFreeRange_;

		double sampleTimeout_;
		double reduceFactor_;

		// obstacle avoidance:
		bool pathRegenOption_;
		bool interactivePathRegen_;
		int pathRegenNum_;

		// target
		std::vector<double> targetRange_;

		// map
		std::shared_ptr<octomap::OcTree> map_;
		double mapRes_;

		// planner
		trajPlanner::pwlTraj* pwlPlanner_;
		globalPlanner::rrtOctomap<3>* rrtPlanner_;

		// visualization
		std::vector<visualization_msgs::Marker> targetVisVec_;
		visualization_msgs::MarkerArray targetVisMsg_; 
		nav_msgs::Path inspectionPath_;
		std::vector<visualization_msgs::Marker> avoidancePathVisVec_;
		visualization_msgs::MarkerArray avoidancePathVisMsg_;


	public:
		std::thread targetVisWorker_;
		std::thread pathVisWorker_;
		std::thread avoidancePathVisWorker_;

		inspector(const ros::NodeHandle& nh);
		void loadParam();
		void initPlanner();
		void run();
		void lookAround(double angle);
		void forward(); // get forward towards the wall
		void forwardNBV(); // forward by Next Best View Criteria
		void moveUp(double height); // move up to the maximum inspection height
		void checkSurroundings(); // check the surrounding dimensions of the target surface
		void inspect(); // generate zig-zag path to inspect the wall
		bool backward(); // go back to the starting position
		bool hasReachTarget(); // check whether the target has been reached
		void mapCB(const octomap_msgs::Octomap &msg);

		// Visualziation
		visualization_msgs::Marker getLineMarker(double x1, double y1, double z1, double x2, double y2, double z2, int id, bool hasReachTarget);
		void updateTargetVis(const std::vector<double>& range, bool hasReachTarget);
		void updatePathVis(const nav_msgs::Path& path);
		void updateAvoidancePathVis(const std::vector<nav_msgs::Path>& pathVec, int bestIdx);
		void publishTargetVis();
		void publishPathVis();
		void publishAvoidancePathVis();

		// helper functions
		geometry_msgs::PoseStamped getForwardGoal(bool& success);
		geometry_msgs::PoseStamped getForwardGoalFromPose(const geometry_msgs::PoseStamped& psTarget, bool& success);
		nav_msgs::Path getForwardPathFromPose(const geometry_msgs::PoseStamped& psTarget, bool& success);
		nav_msgs::Path getForwardPath(bool& success);
		octomap::point3d sampleNBVGoal();
		bool inSensorRange(const octomap::point3d& p, const octomap::point3d& pCheck);
		bool hasOcclusion(const octomap::point3d& p, const octomap::point3d& pCheck);
		int evaluateSample(const octomap::point3d& p);
		bool checkPointSafe(const octomap::point3d& p, double sideSafeReduceFactor=1.0);
		octomap::point3d randomSample(const std::vector<double>& bbox, double& totalReduceFactor);
		octomap::point3d getPoint3dPos();
		std::vector<double> getVecPos();
		geometry_msgs::PoseStamped getCurrPose();
		bool checkCollision(const octomap::point3d &p, bool ignoreUnknown=false);
		bool checkCollisionPoint(const octomap::point3d &p, bool ignoreUnknown=false);
		void setSurroundingFree(const octomap::point3d& p, const std::vector<double>& range);
		void setStartFree();
		double findTargetRangeAxis(const octomap::point3d& pStart, const octomap::point3d& direction, std::vector<octomap::point3d>& resultVec);
		double findTargetRange(std::vector<double>& range);
		octomap::point3d findInspectionStartPoint();
		std::vector<octomap::point3d> getInspectionLimit(const octomap::point3d& p);
		nav_msgs::Path generateZigZagPath();
		geometry_msgs::PoseStamped pointToPose(const octomap::point3d& p); // this will keep current orientation. Be careful
		void moveToPos(const geometry_msgs::Point& position); // only translation movement
		void moveToAngle(const geometry_msgs::Quaternion& quat);
		nav_msgs::Path checkSurroundingsLeft();
		nav_msgs::Path checkSurroundingsRight();
		bool executeWaypointPath(const nav_msgs::Path& path, bool useYaw=false, bool onlineCollisionCheck=true);
		bool executeWaypointPathHeading(const nav_msgs::Path& path, bool onlineCollisionCheck=true);
		bool executeWaypointPathToTime(const nav_msgs::Path& path, double time, geometry_msgs::PoseStamped& psTargetCurr, bool onlineCollisionCheck);
		bool executeAvoidancePath(const nav_msgs::Path& path, bool onlineCollisionCheck=false);
		double poseDistance(const geometry_msgs::PoseStamped& ps1, const geometry_msgs::PoseStamped& ps2);
		double findPathLength(const nav_msgs::Path& path);

		// Collision avoidance
		bool onlineFrontCollisionCheck(); // collision checking in positive x direction
		bool onlineFrontCollisionCheck(double safeDist);
		bool onlineHeadingCollisionCheck();
		void rrtPathRegenInteractive(const std::vector<double>& goalVec, nav_msgs::Path& path);
		void rrtPathRegen(const std::vector<double>& goalVec, nav_msgs::Path& path);
		double evaluatePointObstacleDist(const octomap::point3d& p);
		double evaluatePathMinObstacleDist(const nav_msgs::Path& path);

	};
}



#endif
