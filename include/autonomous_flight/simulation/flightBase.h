/*
	FILE: flightBase.h
	---------------------------------------------
	basic command for quadcopter in simulation
*/

#ifndef FLIGHTBASE_H
#define FLIGHTBASE_H
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/SetModelState.h>
#include <autonomous_flight/simulation/utils.h>
#include <thread>
#include <mutex>

using std::cout; using std::endl;
namespace AutoFlight{
	struct trajData{
		nav_msgs::Path trajectory;
		nav_msgs::Path currTrajectory;
		ros::Time startTime;
		double tCurr;
		double duration;
		double timestep;
		bool init = false;

		void updateTrajectory(const nav_msgs::Path& _trajectory, double _duration){
			this->trajectory = _trajectory;
			this->currTrajectory = _trajectory;
			this->duration = _duration;
			this->tCurr = 0.0;
			this->startTime = ros::Time::now();
			this->timestep = this->duration/(this->trajectory.poses.size()-1);
			if (not this->init){
				this->init = true;
			}
		}

		geometry_msgs::PoseStamped getPose(){
			ros::Time currTime = ros::Time::now();
			this->tCurr = (currTime - this->startTime).toSec() + this->timestep;
			if (this->tCurr > this->duration){
				this->tCurr = this->duration;
			}

			int di = 10;
			int idx = floor(this->tCurr/this->timestep);
			int newIdx = std::min(idx+di, int(this->trajectory.poses.size()-1));
			return this->trajectory.poses[newIdx];
		}
	};

	class flightBase{
	protected:
		// ROS
		ros::NodeHandle nh_;
		ros::Subscriber odomSub_;
		ros::Subscriber clickSub_;
		ros::Publisher takeoffPub_;
		ros::Publisher posePub_;

		nav_msgs::Odometry odom_;
		geometry_msgs::PoseStamped poseTgt_; // target pose
		geometry_msgs::PoseStamped goal_;

		// status
		bool odomReceived_ = false;
		bool hasTakeoff_ = false;
		bool goalReceived_ = false;


	public:
		flightBase(const ros::NodeHandle& nh);
		~flightBase();
		void takeoff();
		void updateTarget(const geometry_msgs::PoseStamped& ps);
		void run();
		void odomCB(const nav_msgs::OdometryConstPtr& odom); 
		void clickCB(const geometry_msgs::PoseStamped::ConstPtr& cp);
		bool isReach(const geometry_msgs::PoseStamped& poseTgt, bool useYaw=true);
	};

	flightBase::flightBase(const ros::NodeHandle& nh) : nh_(nh){
		this->odomSub_ = this->nh_.subscribe("/CERLAB/quadcopter/odom", 10, &flightBase::odomCB, this);
		this->clickSub_ = this->nh_.subscribe("/move_base_simple/goal", 10, &flightBase::clickCB, this);
		this->takeoffPub_ = this->nh_.advertise<std_msgs::Empty>("/CERLAB/quadcopter/takeoff", 1000);
		this->posePub_ = this->nh_.advertise<geometry_msgs::PoseStamped>("/CERLAB/quadcopter/setpoint_pose", 1000);



		ros::Rate r (10);
		while (ros::ok() and not this->odomReceived_){
			ROS_INFO("Wait for odom msg...");
			ros::spinOnce();
			r.sleep();
		}
		ROS_INFO("Topics are ready!");
	}

	flightBase::~flightBase(){}

	void flightBase::takeoff(){
		if (this->odom_.pose.pose.position.z >= 0.2){
			this->hasTakeoff_ = true;
		}

		geometry_msgs::PoseStamped ps;
		ps.pose = this->odom_.pose.pose;
		ps.pose.position.z = 1.0;
		this->updateTarget(ps);
		ros::Rate r (10);
		while (ros::ok() and std::abs(this->odom_.pose.pose.position.z - 1.0) >= 0.1 and not this->hasTakeoff_){
			ros::spinOnce();
			r.sleep();
		}
		this->hasTakeoff_ = true;
	}


	void flightBase::updateTarget(const geometry_msgs::PoseStamped& ps){ // global frame
		this->posePub_.publish(ps);
	}


	void flightBase::run(){
		this->takeoff();
	}


	void flightBase::clickCB(const geometry_msgs::PoseStamped::ConstPtr& cp){
		this->goal_ = *cp;
		this->goal_.pose.position.z = 1.0;
		if (not this->goalReceived_){
			this->goalReceived_ = true;
		}
	}


	void flightBase::odomCB(const nav_msgs::OdometryConstPtr& odom){
		this->odom_ = *odom;
		if (this->odomReceived_ == false){
			this->odomReceived_ = true;
		}
	}


	bool flightBase::isReach(const geometry_msgs::PoseStamped& poseTgt, bool useYaw){
		double targetX, targetY, targetZ, targetYaw, currX, currY, currZ, currYaw;
		targetX = poseTgt.pose.position.x;
		targetY = poseTgt.pose.position.y;
		targetZ = poseTgt.pose.position.z;
		targetYaw = AutoFlight::rpy_from_quaternion(poseTgt.pose.orientation);
		currX = this->odom_.pose.pose.position.x;
		currY = this->odom_.pose.pose.position.y;
		currZ = this->odom_.pose.pose.position.z;
		currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		
		bool reachX, reachY, reachZ, reachYaw;
		reachX = std::abs(targetX - currX) < 0.1;
		reachY = std::abs(targetY - currY) < 0.1;
		reachZ = std::abs(targetZ - currZ) < 0.15;
		if (useYaw){
			reachYaw = std::abs(targetYaw - currYaw) < 0.05;
		}
		else{
			reachYaw = true;
		}
		// cout << reachX << reachY << reachZ << reachYaw << endl;
		if (reachX and reachY and reachZ and reachYaw){
			return true;
		}
		else{
			return false;
		}
	}

}
#endif