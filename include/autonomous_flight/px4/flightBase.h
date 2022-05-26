/*
	FILE: flightBase.h
	-------------------
	base implementation for autonomous flight
*/

#ifndef FLIGHTBASE_H
#define FLIGHTBASE_H
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <thread>
#include <mutex>

namespace AutoFlight{
	class flightBase{
	private:

	protected:
		ros::NodeHandle nh_;
		
		ros::Subscriber stateSub_;
		ros::Subscriber odomSub_;

		ros::Publisher posePub_;

		ros::ServiceClient armClient_;
		ros::ServiceClient setModeClient_;
		
		nav_msgs::Odometry odom_;
		mavros_msgs::State mavrosState_;
		bool odomReady_;
		bool mavrosStateReady_;
		
		geometry_msgs::PoseStamped poseTgt_;


		bool land_ = false;

	public:
		std::thread posePubWorker_;

		flightBase(const ros::NodeHandle& nh);
		~flightBase();
		void updateTarget(const geometry_msgs::PoseStamped& ps);
		void takeoff();
		// void land();
		void run();

		void pubPose();

		// callback
		void stateCB(const mavros_msgs::State::ConstPtr& state);
		void odomCB(const nav_msgs::Odometry::ConstPtr& odom);
	};

	flightBase::flightBase(const ros::NodeHandle& nh) : nh_(nh){
		this->stateSub_ = this->nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &flightBase::stateCB, this);
		this->odomSub_ = this->nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &flightBase::odomCB, this);
		
    	this->armClient_ = this->nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    	this->setModeClient_ = this->nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");	

    	this->odomReady_ = false;
    	this->mavrosStateReady_ = false;
    	ros::Rate r (10);
    	while (ros::ok() and not (this->odomReady_ and this->mavrosStateReady_)){
    		ros::spinOnce();
    		r.sleep();
    	}
    	ROS_INFO("Topics are ready!!");


		this->posePub_ = this->nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
		this->posePubWorker_ = std::thread(&flightBase::pubPose, this);
		this->posePubWorker_.detach();
	}

	flightBase::~flightBase(){}

	void flightBase::updateTarget(const geometry_msgs::PoseStamped& ps){
		this->poseTgt_ = ps;
	}

	void flightBase::takeoff(){
		// from cfg yaml read the flight height
		double takeoffHgt;
		if (not this->nh_.getParam("takeoff_height", takeoffHgt)){
			takeoffHgt = 1.0;
			ROS_INFO("No takeoff_height param found. Use default: 1.0 m.");
		}

		geometry_msgs::PoseStamped ps;
		ps.header.frame_id = "map";
		ps.header.stamp = ros::Time::now();
		ps.pose = this->odom_.pose.pose;
		ps.pose.position.z = takeoffHgt;

		this->updateTarget(ps);
		ROS_INFO("Start taking off...");
		ros::Rate r (10);
		while (ros::ok() and std::abs(this->odom_.pose.pose.position.z - takeoffHgt) >= 0.1){
			r.sleep();
		}
		ROS_INFO("Takeoff succeed!");
	}

	// void flightBase::land(){
	// 	ROS_INFO("Landing...");
	// 	this->land_ = true;
	// 	mavros_msgs::SetMode landMode;
	// 	landMode.request.custom_mode = "AUTO.LAND";
	// 	this->setModeClient_.call(landMode);
	// }

	void flightBase::pubPose(){
		ros::Rate r (10);
		// warmup
		for(int i = 100; ros::ok() && i > 0; --i){
	        this->poseTgt_.header.stamp = ros::Time::now();
	        this->posePub_.publish(this->poseTgt_);
	        ros::spinOnce();
	        r.sleep();
    	}


		mavros_msgs::SetMode offboardMode;
		offboardMode.request.custom_mode = "OFFBOARD";
		mavros_msgs::CommandBool armCmd;
		armCmd.request.value = true;
		ros::Time lastRequest = ros::Time::now();
		while (ros::ok() and not this->land_){
			if (this->mavrosState_.mode != "OFFBOARD" && (ros::Time::now() - lastRequest > ros::Duration(5.0))){
	            if (this->setModeClient_.call(offboardMode) && offboardMode.response.mode_sent){
	                ROS_INFO("Offboard enabled");
	            }
	            lastRequest = ros::Time::now();
	        } else {
	            if (!this->mavrosState_.armed && (ros::Time::now() - lastRequest > ros::Duration(5.0))){
	                if (this->armClient_.call(armCmd) && armCmd.response.success){
	                    ROS_INFO("Vehicle armed");
	                }
	                lastRequest = ros::Time::now();
	            }
	        }

			this->posePub_.publish(this->poseTgt_);
			ros::spinOnce();
			r.sleep();
		}
	}

	void flightBase::run(){
		std::string testMode;
		if (not this->nh_.getParam("test_mode", testMode)){
			testMode = "hover";
			ROS_INFO("No test_mode param found. Use default: hover.");
		}

		if (testMode == "hover"){
			double duration;
			if (not this->nh_.getParam("hover_duration", duration)){
				duration = 5.0;
				ROS_INFO("No hover_duration param found. Use default: 5.0 s.");
			}
			ROS_INFO("Starting hovering for %f second.", duration);
			ros::Time startTime = ros::Time::now();
			ros::Time endTime = ros::Time::now();
			ros::Rate r (10);
			while (ros::ok() and (endTime - startTime <= ros::Duration(duration))){
				endTime = ros::Time::now();
				r.sleep();
			}
		}
		else if (testMode == "circular"){
			double radius;
			if (not this->nh_.getParam("radius", radius)){
				radius = 5.0;
				ROS_INFO("No radius param found. Use default: 2.0 m.");
			}
		}
		else if (testMode == "square"){

		}
		else if (testMode == "eight"){

		}
		else{
			ROS_INFO("Cannot find your test mode!");
		}
	}

	void flightBase::stateCB(const mavros_msgs::State::ConstPtr& state){
		this->mavrosState_ = *state;
		if (not this->mavrosStateReady_){
			this->mavrosStateReady_ = true;
		}
	}

	void flightBase::odomCB(const nav_msgs::Odometry::ConstPtr& odom){
		this->odom_ = *odom;
		if (not this->odomReady_){
			this->odomReady_ = true;
		}
	}
}

#endif