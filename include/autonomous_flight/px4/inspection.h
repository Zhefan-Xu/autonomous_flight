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
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <autonomous_flight/px4/flightBase.h>
#include <math.h>

using std::cout; using std::endl;
namespace AutoFlight{
	class inspector : public flightBase{
	private:
		ros::Subscriber mapSub_;

		std::vector<double> collisionBox_;
		double safeDist_;

		std::shared_ptr<octomap::OcTree> map_;
		double mapRes_;

		// planner
		trajPlanner::pwlTraj* pwlPlanner_;

	public:
		inspector(const ros::NodeHandle& nh);
		void loadParam();
		void initPlanner();
		void run();
		void lookAround();
		void forward(); // get forward towards the wall
		void checkSurroundings(); // check the surrounding dimensions of the target surface
		void inspect(); // generate zig-zag path to inspect the wall
		void backward(); // go back to the starting position
		void mapCB(const octomap_msgs::Octomap &msg);

		// helper functions
		geometry_msgs::PoseStamped getForwardGoal();
		nav_msgs::Path getForwardPath();
		octomap::point3d getPoint3dPos();
		bool checkCollision(const octomap::point3d &p, bool ignoreUnknown=false);
		bool checkCollisionPoint(const octomap::point3d &p, bool ignoreUnknown=false);
		void setSurroundingFree(const octomap::point3d& p);
	};
}



#endif
