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
		this->posePubWorker_ = std::thread(&flightBase::pubPose, this);
		this->posePubWorker_.detach();

		if (not this->nh_.getParam("takeoff_height", this->takeoffHgt_)){
			this->takeoffHgt_ = 1.0;
			ROS_INFO("No takeoff height param found. Use default: 1.0 m.");
		}
		else{
			cout << "[AutoFlight]: Takeoff Height: " << this->takeoffHgt_ << "m." << endl;
		}
	}

	flightBase::~flightBase(){}

	void flightBase::takeoff(){
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
		ros::Rate r (10);
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
	}


	void flightBase::pubPose(){
		ros::Rate r (1.0/this->sampleTime_);
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