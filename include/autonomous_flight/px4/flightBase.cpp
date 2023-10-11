/*
	FILE: flightBase.cpp
	---------------------------
	real world flight implementation
*/
#include <autonomous_flight/px4/flightBase.h>
namespace AutoFlight{
	flightBase::flightBase(const ros::NodeHandle& nh) : nh_(nh){
    	// parameters    	
		if (not this->nh_.getParam("autonomous_flight/takeoff_height", this->takeoffHgt_)){
			this->takeoffHgt_ = 1.0;
			cout << "[AutoFlight]: No takeoff height param found. Use default: 1.0 m." << endl;
		}
		else{
			cout << "[AutoFlight]: Takeoff Height: " << this->takeoffHgt_ << "m." << endl;
		}

		// Subscriber
		this->stateSub_ = this->nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &flightBase::stateCB, this);
		this->odomSub_ = this->nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &flightBase::odomCB, this);
		this->clickSub_ = this->nh_.subscribe("/move_base_simple/goal", 10, &flightBase::clickCB, this);
		
		// Service client
    	this->armClient_ = this->nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    	this->setModeClient_ = this->nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");	

    	// Publisher
		this->posePub_ = this->nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
		this->statePub_ = this->nh_.advertise<tracking_controller::Target>("/autonomous_flight/target_state", 10);


		// Wait for odometry and mavros to be ready
    	this->odomReceived_ = false;
    	this->mavrosStateReceived_ = false;
    	ros::Rate r (10);
    	while (ros::ok() and not (this->odomReceived_ and this->mavrosStateReceived_)){
    		ros::spinOnce();
    		r.sleep();
    	}
    	cout << "[AutoFlight]: Odom and mavros topics are ready." << endl;


    	// Tareget publish thread
		this->targetPubWorker_ = std::thread(&flightBase::publishTarget, this);
		this->targetPubWorker_.detach();

		// state update callback (velocity and acceleration)
		this->stateUpdateTimer_ = this->nh_.createTimer(ros::Duration(0.033), &flightBase::stateUpdateCB, this);	
	}

	void flightBase::publishTarget(){
		ros::Rate r (50);

		// warmup
		for(int i = 100; ros::ok() && i > 0; --i){
	        this->poseTgt_.header.stamp = ros::Time::now();
	        this->posePub_.publish(this->poseTgt_);
    	}

		mavros_msgs::SetMode offboardMode;
		offboardMode.request.custom_mode = "OFFBOARD";
		mavros_msgs::CommandBool armCmd;
		armCmd.request.value = true;
		ros::Time lastRequest = ros::Time::now();
		while (ros::ok()){
			if (this->mavrosState_.mode != "OFFBOARD" && (ros::Time::now() - lastRequest > ros::Duration(5.0))){
	            if (this->setModeClient_.call(offboardMode) && offboardMode.response.mode_sent){
	                cout << "[AutoFlight]: Offboard mode enabled." << endl;
	            }
	            lastRequest = ros::Time::now();
	        } else {
	            if (!this->mavrosState_.armed && (ros::Time::now() - lastRequest > ros::Duration(5.0))){
	                if (this->armClient_.call(armCmd) && armCmd.response.success){
	                    cout << "[AutoFlight]: Vehicle armed." << endl;
	                }
	                lastRequest = ros::Time::now();
	            }
	        }

	        if (this->poseControl_){
	        	this->poseTgt_.header.stamp = ros::Time::now();
	        	this->posePub_.publish(this->poseTgt_);
	        }
	        else{
				this->statePub_.publish(this->stateTgt_);
			}
			// ros::spinOnce();
			r.sleep();
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
		this->currPos_(0) = this->odom_.pose.pose.position.x;
		this->currPos_(1) = this->odom_.pose.pose.position.y;
		this->currPos_(2) = this->odom_.pose.pose.position.z;
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

	void flightBase::stateUpdateCB(const ros::TimerEvent&){
		Eigen::Vector3d currVelBody (this->odom_.twist.twist.linear.x, this->odom_.twist.twist.linear.y, this->odom_.twist.twist.linear.z);
		Eigen::Vector4d orientationQuat (this->odom_.pose.pose.orientation.w, this->odom_.pose.pose.orientation.x, this->odom_.pose.pose.orientation.y, this->odom_.pose.pose.orientation.z);
		Eigen::Matrix3d orientationRot = AutoFlight::quat2RotMatrix(orientationQuat);
		this->currVel_ = orientationRot * currVelBody;	
		ros::Time currTime = ros::Time::now();	
		if (this->stateUpdateFirstTime_){
			this->currAcc_ = Eigen::Vector3d (0.0, 0.0, 0.0);
			this->prevStateTime_ = currTime;
			this->stateUpdateFirstTime_ = false;
		}
		else{
			double dt = (currTime - this->prevStateTime_).toSec();
			this->currAcc_ = (this->currVel_ - this->prevVel_)/dt;
			this->prevVel_ = this->currVel_; 
			this->prevStateTime_ = currTime;
		}
	}

	void flightBase::takeoff(){
		// from cfg yaml read the flight height
		geometry_msgs::PoseStamped ps;
		ps.header.frame_id = "map";
		ps.header.stamp = ros::Time::now();
		ps.pose.position.x = this->odom_.pose.pose.position.x;
		ps.pose.position.y = this->odom_.pose.pose.position.y;
		ps.pose.position.z = this->takeoffHgt_;
		ps.pose.orientation = this->odom_.pose.pose.orientation;
		this->updateTarget(ps);


		cout << "[AutoFlight]: Start taking off..." << endl;
		ros::Rate r (30);
		while (ros::ok() and std::abs(this->odom_.pose.pose.position.z - this->takeoffHgt_) >= 0.1){
			ros::spinOnce();
			r.sleep();
		}

		// tracking_controller::Target psT;
		// // psT.type_mask = psT.IGNORE_ACC_VEL;
		// psT.header.frame_id = "map";
		// psT.header.stamp = ros::Time::now();
		// psT.position.x = this->odom_.pose.pose.position.x;
		// psT.position.y = this->odom_.pose.pose.position.y;
		// psT.position.z = this->takeoffHgt_;
		// psT.yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		// this->updateTargetWithState(psT);
		
		// cout << "[AutoFlight]: Switch to tracking controller." << endl;
		ros::Time startTime = ros::Time::now();
		while (ros::ok()){
			ros::Time currTime = ros::Time::now();
			if ((currTime - startTime).toSec() >= 3){
				break;
			}
			ros::spinOnce();
			r.sleep();
		}
		cout << "[AutoFlight]: Takeoff succeed!" << endl;
	}

	void flightBase::run(){
		// flight test with circle
		double r = 2.0; // radius
		double v = 3.0; // 2.0 m/s

		double z = this->odom_.pose.pose.position.z;
		geometry_msgs::PoseStamped startPs;
		startPs.pose.position.x = r;
		startPs.pose.position.y = 0.0;
		startPs.pose.position.z = z;
		this->updateTarget(startPs);
		
		cout << "[AutoFlight]: Go to target point..." << endl;
		ros::Rate rate (30);
		while (ros::ok() and std::abs(this->odom_.pose.pose.position.x - startPs.pose.position.x) >= 0.1){
			ros::spinOnce();
			rate.sleep();
		}
		cout << "[AutoFlight]: Reach target point." << endl;

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
			ros::spinOnce();
			rate.sleep();
		}
	}

	void flightBase::stop(){
		geometry_msgs::PoseStamped ps;
		ps.pose = this->odom_.pose.pose;
		this->updateTarget(ps);
	}

	void flightBase::moveToOrientation(double yaw, double desiredAngularVel){
		double yawTgt = yaw;
		geometry_msgs::Quaternion orientation = AutoFlight::quaternion_from_rpy(0, 0, yaw);
		double yawCurr = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);		
		geometry_msgs::PoseStamped ps;
		ps.pose = this->odom_.pose.pose;
		ps.pose.orientation = orientation;

		double yawDiff = yawTgt - yawCurr; // difference between yaw
		double direction = 0;
		double yawDiffAbs = std::abs(yawDiff);
		if ((yawDiffAbs <= PI_const) and (yawDiff>0)){
			direction = 1.0; // counter clockwise
		} 
		else if ((yawDiffAbs <= PI_const) and (yawDiff<0)){
			direction = -1.0; // clockwise
		}
		else if ((yawDiffAbs > PI_const) and (yawDiff>0)){
			direction = -1.0; // rotate in clockwise direction
			yawDiffAbs = 2 * PI_const - yawDiffAbs;
		}
		else if ((yawDiffAbs > PI_const) and (yawDiff<0)){
			direction = 1.0; // counter clockwise
			yawDiffAbs = 2 * PI_const - yawDiffAbs;
		}

		double endTime = yawDiffAbs/desiredAngularVel;
		geometry_msgs::PoseStamped psT;
		psT.pose = ps.pose;
		ros::Time startTime = ros::Time::now();
		ros::Time currTime = ros::Time::now();
		ros::Rate r (30);
		while (ros::ok() and not this->isReach(ps)){
			currTime = ros::Time::now();
			double t = (currTime - startTime).toSec();

			if (t >= endTime){ 
				psT = ps;
			}
			else{
				double currYawTgt = yawCurr + (double) direction * t/endTime * yawDiffAbs;
				geometry_msgs::Quaternion quatT = AutoFlight::quaternion_from_rpy(0, 0, currYawTgt);
				psT.pose.orientation = quatT;
				
			}
			this->updateTarget(psT);
			// cout << "here" << endl;
			ros::spinOnce();
			r.sleep();
		}
	}

	void flightBase::updateTarget(const geometry_msgs::PoseStamped& ps){
		this->poseTgt_ = ps;
		this->poseTgt_.header.frame_id = "map";
		this->poseControl_ = true;
	}

	void flightBase::updateTargetWithState(const tracking_controller::Target& target){
		this->stateTgt_ = target;
		this->poseControl_ = false;
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

		if (reachX and reachY and reachZ and reachYaw){
			return true;
		}
		else{
			return false;
		}
	}

	bool flightBase::isReach(const geometry_msgs::PoseStamped& poseTgt, double dist, bool useYaw){
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
		reachX = std::abs(targetX - currX) < dist;
		reachY = std::abs(targetY - currY) < dist;
		reachZ = std::abs(targetZ - currZ) < dist;
		if (useYaw){
			reachYaw = std::abs(targetYaw - currYaw) < 0.05;
		}
		else{
			reachYaw = true;
		}

		if (reachX and reachY and reachZ and reachYaw){
			return true;
		}
		else{
			return false;
		}
	}
}