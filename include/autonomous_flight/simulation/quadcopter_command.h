/*
	FILE: quadcopter_command.h
	basic command for quadcopter in simulation
*/

#ifndef QUADCOPTERCOMMAND_H
#define QUADCOPTERCOMMAND_H
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/SetModelState.h>
#include <autonomous_flight/simulation/utils.h>


namespace AutoFlight{
	class quadCommand{
	private:
		ros::NodeHandle nh_;
		
		ros::Publisher takeoffPub_;
		ros::Publisher landPub_;
		ros::Publisher velPub_;
		ros::Publisher posPub_;

		ros::Subscriber odomSub_;

		ros::ServiceClient setStateClient_;

		nav_msgs::Odometry odom_;

	public:
		quadCommand(const ros::NodeHandle& nh);
		void takeoff();
		void land();
		void setVelocity(double vx, double vy, double vz);
		void setPosition(double x, double y, double z, double yaw=0);
		void resetPosition(double x, double y, double z=0, double yaw=0);
		void odomCB(const nav_msgs::OdometryConstPtr& odom); 
	};


	quadCommand::quadCommand(const ros::NodeHandle& nh) : nh_(nh){
		this->takeoffPub_ = this->nh_.advertise<std_msgs::Empty>("/CERLAB/quadcopter/takeoff", 1000);
		this->landPub_ = this->nh_.advertise<std_msgs::Empty>("/CERLAB/quadcopter/land", 1000);
		this->velPub_ = this->nh_.advertise<geometry_msgs::TwistStamped>("/CERLAB/quadcopter/cmd_vel", 1000);

		this->odomSub_ = this->nh_.subscribe("/CERLAB/quadcopter/odom", 10, &quadCommand::odomCB, this);
		
		this->setStateClient_ = this->nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	}

	void quadCommand::takeoff(){
		std_msgs::Empty msg;
		ros::Rate r (10);
		while (ros::ok() and this->odom_.pose.pose.position.z < 0.5){
			this->takeoffPub_.publish(msg);
			ros::spinOnce();
			r.sleep();
		}
	}

	void quadCommand::land(){
		std_msgs::Empty msg;
		ros::Rate r (10);
		while (ros::ok() and this->odom_.pose.pose.position.z >= 0.5){
			this->landPub_.publish(msg);
			ros::spinOnce();
			r.sleep();
		}
	}

	void quadCommand::setVelocity(double vx, double vy, double vz){ // local frame
		geometry_msgs::TwistStamped ts;
		ts.header.frame_id = "base_link";
		ts.header.stamp = ros::Time::now();
		ts.twist.linear.x = vx;
		ts.twist.linear.y = vy;
		ts.twist.linear.z = vz;
		this->velPub_.publish(ts);
	}

	void quadCommand::setPosition(double x, double y, double z, double yaw){ // global frame
		
	}

	void quadCommand::resetPosition(double x, double y, double z, double yaw){
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

	void quadCommand::odomCB(const nav_msgs::OdometryConstPtr& odom){
		this->odom_ = *odom;
	}
}
#endif