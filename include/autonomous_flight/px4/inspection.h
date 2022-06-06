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
		ros::Publisher targetVisPub_;
		ros::Publisher pathPub_; // visualization

		// parameters
		std::vector<double> collisionBox_;
		double safeDist_;
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
		double sampleTimeout_;
		double reduceFactor_;

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

	public:
		std::thread targetVisWorker_;
		std::thread pathVisWorker_;

		inspector(const ros::NodeHandle& nh);
		void loadParam();
		void initPlanner();
		void run();
		void lookAround();
		bool forward(); // get forward towards the wall
		void forwardNBV(); // forward by Next Best View Criteria
		void moveUp(double height); // move up to the maximum inspection height
		void checkSurroundings(); // check the surrounding dimensions of the target surface
		void inspect(); // generate zig-zag path to inspect the wall
		void backward(); // go back to the starting position
		bool hasReachTarget(); // check whether the target has been reached
		void mapCB(const octomap_msgs::Octomap &msg);

		// Visualziation
		visualization_msgs::Marker getLineMarker(double x1, double y1, double z1, double x2, double y2, double z2, int id, bool hasReachTarget);
		void updateTargetVis(const std::vector<double>& range, bool hasReachTarget);
		void updatePathVis(const nav_msgs::Path& path);
		void publishTargetVis();
		void publishPathVis();

		// helper functions
		geometry_msgs::PoseStamped getForwardGoal(bool& success);
		nav_msgs::Path getForwardPath(bool& success);
		octomap::point3d sampleNBVGoal();
		bool inSensorRange(const octomap::point3d& p, const octomap::point3d& pCheck);
		bool hasOcclusion(const octomap::point3d& p, const octomap::point3d& pCheck);
		int evaluateSample(const octomap::point3d& p);
		bool checkPointSafe(const octomap::point3d& p, double sideSafeReduceFactor=1.0);
		octomap::point3d randomSample(const std::vector<double>& bbox);
		octomap::point3d getPoint3dPos();
		std::vector<double> getVecPos();
		bool checkCollision(const octomap::point3d &p, bool ignoreUnknown=false);
		bool checkCollisionPoint(const octomap::point3d &p, bool ignoreUnknown=false);
		void setSurroundingFree(const octomap::point3d& p);
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
	};
}



#endif
