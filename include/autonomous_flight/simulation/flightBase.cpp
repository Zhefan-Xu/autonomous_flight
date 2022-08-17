/*
	FILE: flightBase.h
	-------------------------
	implementation of flight base
*/
#include <autonomous_flight/simulation/flightBase.h>


namespace AutoFlight{
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
		if (not this->firstGoal_){
			this->firstGoal_ = true;
		}

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