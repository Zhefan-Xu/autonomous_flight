/*
	FILE: flightBase.h
	basic command for quadcopter in simulation
*/

#ifndef FLIGHTBASE_H
#define FLIGHTBASE_H
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/SetModelState.h>
#include <autonomous_flight/simulation/utils.h>
#include <thread>
#include <mutex>

namespace AutoFlight{
	class flightBase{
	private:
		ros::NodeHandle nh_;
		
		ros::Publisher takeoffPub_;
		ros::Publisher landPub_;
		ros::Publisher velPub_;
		ros::Publisher posePub_;
		ros::Publisher goalPub_;

		ros::Subscriber odomSub_;
		ros::Subscriber clickSub_;

		ros::ServiceClient setStateClient_;

		nav_msgs::Odometry odom_;

		// for click
		bool clickGoal_;
		bool clickGoalInit_;
		int clickCount_;
		std::vector<double> clickGoalPos_;
		geometry_msgs::PoseStamped goalMsg_;

		// status
		bool odomReceived_ = false;
		bool hasTakeoff_ = false;



	public:
		std::thread goalPubWorker_;

		flightBase(const ros::NodeHandle& nh);
		~flightBase();
		void takeoff();
		void land();
		void setVelocity(double vx, double vy, double vz);
		void setPose(double x, double y, double z, double yaw=0);
		void setPose(const geometry_msgs::PoseStamped& ps);
		void resetPose(double x, double y, double z=0, double yaw=0);
		void switchClickMode(bool clickGoal);
		void pubGoal();
		std::vector<double> getGoalClick();
		std::vector<double> getPosition();
		nav_msgs::Odometry getOdom();
		void odomCB(const nav_msgs::OdometryConstPtr& odom); 
		void clickCB(const geometry_msgs::PoseStamped::ConstPtr& cp);
		bool isReach(const geometry_msgs::PoseStamped& poseTgt, bool useYaw=true);
	};


	flightBase::flightBase(const ros::NodeHandle& nh) : nh_(nh){
		this->takeoffPub_ = this->nh_.advertise<std_msgs::Empty>("/CERLAB/quadcopter/takeoff", 1000);
		this->landPub_ = this->nh_.advertise<std_msgs::Empty>("/CERLAB/quadcopter/land", 1000);
		this->velPub_ = this->nh_.advertise<geometry_msgs::TwistStamped>("/CERLAB/quadcopter/cmd_vel", 1000);
		this->posePub_ = this->nh_.advertise<geometry_msgs::PoseStamped>("/CERLAB/quadcopter/setpoint_pose", 1000);
		this->goalPub_ = this->nh_.advertise<geometry_msgs::PoseStamped>("/quadCommand/goal", 1000);

		this->odomSub_ = this->nh_.subscribe("/CERLAB/quadcopter/odom", 10, &flightBase::odomCB, this);
		this->clickSub_ = this->nh_.subscribe("/move_base_simple/goal", 10, &flightBase::clickCB, this);

		this->setStateClient_ = this->nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
		this->clickGoalInit_ = false;
		this->clickGoal_ = true;
		this->clickCount_ = 0;


		this->goalPubWorker_ = std::thread(&flightBase::pubGoal, this);

		ros::Rate r (1);
		while (ros::ok() and not this->odomReceived_){
			ROS_INFO("wait for odom msg...");
			ros::spinOnce();
			r.sleep();
		}
	}

	flightBase::~flightBase(){
		this->goalPubWorker_.detach();
	}

	void flightBase::takeoff(){
		if (this->odom_.pose.pose.position.z >= 0.2){
			this->hasTakeoff_ = false;
		}

		std_msgs::Empty msg;
		ros::Rate r (10);
		while (ros::ok() and std::abs(this->odom_.pose.pose.position.z - 1.0) >= 0.1 and not this->hasTakeoff_){
			// this->takeoffPub_.publish(msg);
			this->setPose(this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, 1.0, rpy_from_quaternion(this->odom_.pose.pose.orientation));
			ros::spinOnce();
			r.sleep();
		}
		this->hasTakeoff_ = false;
	}

	void flightBase::land(){
		std_msgs::Empty msg;
		ros::Rate r (10);
		while (ros::ok() and this->odom_.pose.pose.position.z >= 0.5){
			this->landPub_.publish(msg);
			ros::spinOnce();
			r.sleep();
		}
	}

	void flightBase::setVelocity(double vx, double vy, double vz){ // local frame
		geometry_msgs::TwistStamped ts;
		ts.header.frame_id = "base_link";
		ts.header.stamp = ros::Time::now();
		ts.twist.linear.x = vx;
		ts.twist.linear.y = vy;
		ts.twist.linear.z = vz;
		this->velPub_.publish(ts);
	}

	void flightBase::setPose(double x, double y, double z, double yaw){ // global frame
		geometry_msgs::PoseStamped ps;
		ps.pose.position.x = x;
		ps.pose.position.y = y;
		ps.pose.position.z = z;
		ps.pose.orientation = quaternion_from_rpy(0, 0, yaw);
		this->posePub_.publish(ps);
	}

	void flightBase::setPose(const geometry_msgs::PoseStamped& ps){ // global frame
		this->posePub_.publish(ps);
	}

	void flightBase::resetPose(double x, double y, double z, double yaw){
		gazebo_msgs::SetModelState srv;
		gazebo_msgs::ModelState msg;
		msg.model_name = "quadcopter";
		msg.pose.position.x = x;
		msg.pose.position.y = y;
		msg.pose.position.z = z;
		msg.pose.orientation = quaternion_from_rpy(0, 0, yaw);
		msg.reference_frame = "map";
		srv.request.model_state = msg;
		this->setStateClient_.call(srv);
	}

	void flightBase::switchClickMode(bool clickGoal){
		this->clickGoal_ = clickGoal;
		this->clickGoalInit_ = false;
	}


	void flightBase::pubGoal(){
		ros::Rate r (10);
		while (ros::ok() and this->clickGoalInit_ == false){
			ros::spinOnce();
			r.sleep();
		}

		while (ros::ok() and this->clickGoalInit_){
			this->goalMsg_.header.stamp = ros::Time::now();
			this->goalPub_.publish(this->goalMsg_);
			r.sleep();
		}
	}

	std::vector<double> flightBase::getPosition(){
		std::vector<double> pos {this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z};
		return pos;
	}


	std::vector<double> flightBase::getGoalClick(){
		ros::Rate r (10);
		while (ros::ok() and (this->clickGoalInit_ == false or this->clickCount_ % 2 != 0)){
			ROS_INFO("Wait for clicked goal and start position...");
			ros::spinOnce();
			r.sleep();
		}
		this->clickGoalInit_ = false;
		this->clickCount_ = 0;
		return this->clickGoalPos_;
	}

	nav_msgs::Odometry flightBase::getOdom(){
		return this->odom_;
	}

	void flightBase::odomCB(const nav_msgs::OdometryConstPtr& odom){
		this->odom_ = *odom;
		if (this->odomReceived_ == false){
			this->odomReceived_ = true;
		}
	}

	void flightBase::clickCB(const geometry_msgs::PoseStamped::ConstPtr& cp){
		double x, y, z;
		x = (*cp).pose.position.x;
		y = (*cp).pose.position.y;
		z = 1.0;
		if (not this->clickGoal_){
			if (this->clickCount_ % 2 == 0){
				this->clickGoalPos_ = std::vector<double> {x, y, z}; 
				this->goalMsg_ = *cp;
				this->goalMsg_.pose.position.x = x;
				this->goalMsg_.pose.position.y = y;
				this->goalMsg_.pose.position.z = 1.0;
				if (this->clickGoalInit_ ==  false){  
					this->clickGoalInit_ = true;
				}
			}
			else{
				double yaw = atan2(this->clickGoalPos_[1]-y, this->clickGoalPos_[0]-x);
				this->resetPose(x, y, z, yaw);
			}
		}
		else{
			this->clickGoalPos_ = std::vector<double> {x, y, z}; 
			this->goalMsg_ = *cp;
			this->goalMsg_.pose.position.x = x;
			this->goalMsg_.pose.position.y = y;
			this->goalMsg_.pose.position.z = 1.0;
			if (this->clickGoalInit_ ==  false){  
				this->clickGoalInit_ = true;
			}
			++this->clickCount_;
		}
		++this->clickCount_;
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