/*
	FILE: flightBase.h
	-------------------
	base implementation for autonomous flight
*/

#ifndef FLIGHTBASE_H
#define FLIGHTBASE_H
#include <ros/ros.h>
#include <autonomous_flight/px4/utils.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tracking_controller/Target.h>
#include <Eigen/Dense>
#include <thread>
#include <mutex>

using std::cout; using std::endl;
namespace AutoFlight{
	class flightBase{
	protected:
		ros::NodeHandle nh_;
		ros::Subscriber stateSub_;
		ros::Subscriber odomSub_;
		ros::Subscriber clickSub_;
		ros::Publisher posePub_;
		ros::Publisher statePub_;
		ros::ServiceClient armClient_;
		ros::ServiceClient setModeClient_;
		ros::Timer targetPubTimer_;
		
		nav_msgs::Odometry odom_;
		mavros_msgs::State mavrosState_;
		geometry_msgs::PoseStamped poseTgt_;
		tracking_controller::Target stateTgt_;
		geometry_msgs::PoseStamped goal_;
		
		// parameters
		double takeoffHgt_;

		// status
		bool poseControl_ = true;
		bool odomReceived_ = false;
		bool mavrosStateReceived_ = false;
		bool firstGoal_ = false;
		bool goalReceived_ = false;


	public:
		std::thread targetPubWorker_;

		flightBase(const ros::NodeHandle& nh);
		
		void publishTarget();

		// callback functions
		void stateCB(const mavros_msgs::State::ConstPtr& state);
		void odomCB(const nav_msgs::Odometry::ConstPtr& odom);
		void clickCB(const geometry_msgs::PoseStamped::ConstPtr& cp);

		void takeoff();
		void run(); // in flight base, this is a trajectory test function
		void stop(); // stop at the current position
		void moveToOrientation(double yaw, double desiredAngularVel);

		void updateTarget(const geometry_msgs::PoseStamped& ps);
		void updateTargetWithState(const tracking_controller::Target& target);
		bool isReach(const geometry_msgs::PoseStamped& poseTgt, bool useYaw=true);
	};
}

#endif
