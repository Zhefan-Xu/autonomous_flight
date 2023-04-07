/*
	FILE: flightBase.cpp
	---------------------------
	real world flight implementation
*/
#include <autonomous_flight/px4/flightBase.h>
namespace AutoFlight{
	flightBase::flightBase(const ros::NodeHandle& nh) : nh_(nh){
		this->stateSub_ = this->nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &flightBase::stateCB, this);
		this->odomSub_ = this->nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &flightBase::odomCB, this);
		this->clickSub_ = this->nh_.subscribe("/move_base_simple/goal", 10, &flightBase::clickCB, this);
		
    	this->armClient_ = this->nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    	this->setModeClient_ = this->nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");	

    	this->odomReceived_ = false;
    	this->mavrosStateReceived_ = false;
    	ros::Rate r (10);
    	while (ros::ok() and not (this->odomReceived_ and this->mavrosStateReceived_)){
    		ros::spinOnce();
    		r.sleep();
    	}
    	ROS_INFO("Topics are ready!!");


    	// pose publisher
    	if (not this->nh_.getParam("sample_time", this->sampleTime_)){
    		this->sampleTime_ = 0.1;
    		ROS_INFO("No sample time param. Use default: 0.1s.");
    	}

		this->posePub_ = this->nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1000);
		this->statePub_ = this->nh_.advertise<tracking_controller::Target>("/autonomous_flight/target_state", 1000);
		this->statePubWorker_ = std::thread(&flightBase::pubState, this);
		this->statePubWorker_.detach();
		

		if (not this->nh_.getParam("takeoff_height", this->takeoffHgt_)){
			this->takeoffHgt_ = 1.0;
			ROS_INFO("No takeoff height param found. Use default: 1.0 m.");
		}
		else{
			cout << "[AutoFlight]: Takeoff Height: " << this->takeoffHgt_ << "m." << endl;
		}

		if (not this->nh_.getParam("pose_control", this->poseControl_)){
			this->poseControl_ = false;
			ROS_INFO("No pose control param found. Use default: state control.");
		}
		else{
			cout << "[AutoFlight]: Pose control (1)/State control(0): " << this->poseControl_ << endl;
		}
	}

	flightBase::~flightBase(){}

	void flightBase::takeoff(){
		// if (this->poseControl_){
		// 	// from cfg yaml read the flight height
		// 	geometry_msgs::PoseStamped ps;
		// 	ps.header.frame_id = "map";
		// 	ps.header.stamp = ros::Time::now();
		// 	ps.pose.position.x = 0.0;
		// 	ps.pose.position.y = 0.0;
		// 	// ps.pose.position.x = this->odom_.pose.pose.position.x;
		// 	// ps.pose.position.y = this->odom_.pose.pose.position.y;
		// 	ps.pose.position.z = this->takeoffHgt_;
		// 	ps.pose.orientation.x = 0.0;
		// 	ps.pose.orientation.y = 0.0;
		// 	ps.pose.orientation.z = 0.0;
		// 	ps.pose.orientation.w = 1.0;
		// 	this->updateTarget(ps);
		// }
		// else{
		// 	tracking_controller::Target tgt;
		// 	tgt.header.frame_id = "map";
		// 	tgt.header.stamp = ros::Time::now();
		// 	tgt.position.x = 0.0;
		// 	tgt.position.y = 0.0;
		// 	tgt.position.z = this->takeoffHgt_;
		// 	tgt.velocity.x = 0.0;
		// 	tgt.velocity.y = 0.0;
		// 	tgt.velocity.z = 0.0;
		// 	tgt.acceleration.x = 0.0;
		// 	tgt.acceleration.y = 0.0;
		// 	tgt.acceleration.z = 0.0;
		// 	tgt.yaw = 0.0;
		// 	this->updateTargetWithState(tgt);
		// }

		// from cfg yaml read the flight height
		geometry_msgs::PoseStamped ps;
		ps.header.frame_id = "map";
		ps.header.stamp = ros::Time::now();
		ps.pose.position.x = 0.0;
		ps.pose.position.y = 0.0;
		// ps.pose.position.x = this->odom_.pose.pose.position.x;
		// ps.pose.position.y = this->odom_.pose.pose.position.y;
		ps.pose.position.z = this->takeoffHgt_;
		ps.pose.orientation.x = 0.0;
		ps.pose.orientation.y = 0.0;
		ps.pose.orientation.z = 0.0;
		ps.pose.orientation.w = 1.0;
		this->updateTarget(ps);
		ROS_INFO("Start taking off...");
		ros::Rate r (50);
		while (ros::ok() and std::abs(this->odom_.pose.pose.position.z - this->takeoffHgt_) >= 0.1){
			ros::spinOnce();
			r.sleep();
		}
		ROS_INFO("Takeoff succeed!");
	}

	void flightBase::updateTarget(const geometry_msgs::PoseStamped& ps){
		this->poseTgt_ = ps;
		this->poseTgt_.header.frame_id = "map";
		this->poseTgt_.header.stamp = ros::Time::now();
		this->poseControl_ = true;
	}

	void flightBase::updateTargetWithState(const tracking_controller::Target& target){
		this->stateTgt_ = target;
		this->poseControl_ = false;
	}


	void flightBase::pubState(){
		ros::Rate r (1.0/this->sampleTime_);

		mavros_msgs::SetMode offboardMode;
		offboardMode.request.custom_mode = "OFFBOARD";
		mavros_msgs::CommandBool armCmd;
		armCmd.request.value = true;
		ros::Time lastRequest = ros::Time::now();
		while (ros::ok()){
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

	        if (this->poseControl_){
	        	this->posePub_.publish(this->poseTgt_);
	        }
	        else{
				this->statePub_.publish(this->stateTgt_);
			}
			r.sleep();
		}
	}

	void flightBase::run(){
		double z = this->odom_.pose.pose.position.z;
		// go to (1, 0, height) point
		geometry_msgs::PoseStamped startPs;
		startPs.pose.position.x = 2.0;
		startPs.pose.position.y = 0.0;
		startPs.pose.position.z = z;
		this->updateTarget(startPs);
		ROS_INFO("Go to start point...");
		ros::Rate r1 (50);
		while (ros::ok() and std::abs(this->odom_.pose.pose.position.x - startPs.pose.position.x) >= 0.1){
			ros::spinOnce();
			r1.sleep();
		}
		ROS_INFO("Start point reached!");


		// flight test with circle
		double r = 2.0; // radius
		double v = 2.0; // 2.0 m
		ros::Publisher targetPosePub = this->nh_.advertise<geometry_msgs::PoseStamped>("/autonomous_flight/test_target", 10);
		ros::Rate rate (50);
		ros::Time startTime = ros::Time::now();
		while (ros::ok()){
			ros::Time currTime = ros::Time::now();
			double t = (currTime - startTime).toSec();
			double rad = v * t / r;
			double x = r * cos(rad);
			double y = r * sin(rad);
			double vx = -v * sin(rad);
			double vy = v * cos(rad);
			double vz = 0.0;
			double aNorm = v*v/r;
			Eigen::Vector3d accVec (x, y, 0);
			accVec = -aNorm * accVec / accVec.norm();
			double ax = accVec(0);
			double ay = accVec(1);
			double az = 0.0;

			// state target message
			tracking_controller::Target target;
			target.position.x = x;
			target.position.y = y;
			target.position.z = z;
			target.velocity.x = vx;
			target.velocity.y = vy;
			target.velocity.z = vz;
			target.acceleration.x = ax;
			target.acceleration.y = ay;
			target.acceleration.z = az;
			this->updateTargetWithState(target);


			geometry_msgs::PoseStamped ps;
			ps.header.frame_id = "map";
			ps.header.stamp = ros::Time::now();
			ps.pose.position.x = x;
			ps.pose.position.y = y;
			ps.pose.position.z = z;
			targetPosePub.publish(ps);
			rate.sleep();
		}
	}

	void flightBase::stateCB(const mavros_msgs::State::ConstPtr& state){
		this->mavrosState_ = *state;
		if (not this->mavrosStateReceived_){
			this->mavrosStateReceived_ = true;
		}
	}

	void flightBase::odomCB(const nav_msgs::Odometry::ConstPtr& odom){
		this->odom_ = *odom;
		if (not this->odomReceived_){
			this->odomReceived_ = true;
		}
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