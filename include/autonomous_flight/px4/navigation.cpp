/*
	FILE: navigation.cpp
	------------------------
	navigation implementation file in real world
*/
#include <autonomous_flight/px4/navigation.h>

namespace AutoFlight{
	navigation::navigation(const ros::NodeHandle& nh) : flightBase(nh){
		this->initParam();
		this->initModules();
		this->registerPub();
	}

	void navigation::initParam(){
    	// parameters    
    	// use global planner or not	
		if (not this->nh_.getParam("autonomous_flight/use_global_planner", this->useGlobalPlanner_)){
			this->useGlobalPlanner_ = false;
			cout << "[AutoFlight]: No use global planner param found. Use default: false." << endl;
		}
		else{
			cout << "[AutoFlight]: Global planner use is set to: " << this->useGlobalPlanner_ << "." << endl;
		}

		// full state control (yaw)
		if (not this->nh_.getParam("autonomous_flight/use_yaw_control", this->useYawControl_)){
			this->useYawControl_ = false;
			cout << "[AutoFlight]: No yaw control param found. Use default: false." << endl;
		}
		else{
			cout << "[AutoFlight]: Yaw control use is set to: " << this->useYawControl_ << "." << endl;
		}		

    	// desired linear velocity    	
		if (not this->nh_.getParam("autonomous_flight/desired_velocity", this->desiredVel_)){
			this->desiredVel_ = 1.0;
			cout << "[AutoFlight]: No desired velocity param found. Use default: 1.0 m/s." << endl;
		}
		else{
			cout << "[AutoFlight]: Desired velocity is set to: " << this->desiredVel_ << "m/s." << endl;
		}

    	// desired angular velocity    	
		if (not this->nh_.getParam("autonomous_flight/desired_angular_velocity", this->desiredAngularVel_)){
			this->desiredAngularVel_ = 1.0;
			cout << "[AutoFlight]: No desired angular velocity param found. Use default: 0.5 rad/s." << endl;
		}
		else{
			cout << "[AutoFlight]: Desired angular velocity is set to: " << this->desiredAngularVel_ << "rad/s." << endl;
		}		
	}

	void navigation::initModules(){
		// initialize map
		this->map_.reset(new mapManager::occMap (this->nh_));

		// initialize rrt planner
		this->rrtPlanner_.reset(new globalPlanner::rrtOccMap<3> (this->nh_));
		this->rrtPlanner_->setMap(this->map_);

		// initialize polynomial trajectory planner
		this->polyTraj_.reset(new trajPlanner::polyTrajOccMap (this->nh_));
		this->polyTraj_->setMap(this->map_);

		// initialize piecewise linear trajectory planner
		this->pwlTraj_.reset(new trajPlanner::pwlTraj (this->nh_));

		// initialize bspline trajectory planner
		this->bsplineTraj_.reset(new trajPlanner::bsplineTraj (this->nh_));
		this->bsplineTraj_->setMap(this->map_);
	}

	void navigation::registerPub(){
		this->rrtPathPub_ = this->nh_.advertise<nav_msgs::Path>("navigation/rrt_path", 10);
		this->polyTrajPub_ = this->nh_.advertise<nav_msgs::Path>("navigation/poly_traj", 10);
		this->pwlTrajPub_ = this->nh_.advertise<nav_msgs::Path>("navigation/pwl_trajectory", 10);
		this->bsplineTrajPub_ = this->nh_.advertise<nav_msgs::Path>("navigation/bspline_trajectory", 10);
		this->inputTrajPub_ = this->nh_.advertise<nav_msgs::Path>("navigation/input_trajectory", 10);
	}

	void navigation::registerCallback(){
		// planner callback
		this->plannerTimer_ = this->nh_.createTimer(ros::Duration(0.02), &navigation::plannerCB, this);

		// collision check callback
		this->replanCheckTimer_ = this->nh_.createTimer(ros::Duration(0.01), &navigation::replanCheckCB, this);

		// trajectory execution callback
		this->trajExeTimer_ = this->nh_.createTimer(ros::Duration(0.01), &navigation::trajExeCB, this);

		// state update callback (velocity and acceleration)
		this->stateUpdateTimer_ = this->nh_.createTimer(ros::Duration(0.033), &navigation::stateUpdateCB, this);

		// visualization callback
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.033), &navigation::visCB, this);
	}

	void navigation::plannerCB(const ros::TimerEvent&){
		if (not this->firstGoal_) return;

		if (this->useGlobalPlanner_){
			// only if a new goal is received, the global planner will make plan
			if (this->goalReceived_){ 
			}
		}

		if (this->replan_){
			double sampleTime = 0.1;
			// piecewise linear trajectory
			nav_msgs::Path waypoints;
			geometry_msgs::PoseStamped start, goal;
			start.pose = this->odom_.pose.pose;
			goal = this->goal_;
			waypoints.poses = std::vector<geometry_msgs::PoseStamped> {start, goal};
			this->pwlTraj_->updatePath(waypoints, this->desiredVel_);
			this->pwlTraj_->makePlan(this->pwlTrajMsg_, sampleTime);




			// bspline trajectory generation
			nav_msgs::Path inputTraj;
			if (not this->trajectoryReady_){
				inputTraj = this->pwlTrajMsg_;
			}
			else{
				inputTraj = this->getCurrentTraj(sampleTime);
				// check the distance between last point and the goal position
				if (AutoFlight::getPoseDistance(inputTraj.poses.back(), this->goal_) >= 0.2){
					// use polynomial trajectory to make the rest of the trajectory
					nav_msgs::Path waypoints, polyTrajTemp;
					waypoints.poses = std::vector<geometry_msgs::PoseStamped>{inputTraj.poses.back(), this->goal_};
					std::vector<Eigen::Vector3d> polyStartEndCondition;
					Eigen::Vector3d polyStartVel = this->trajectory_.getDerivative().at(this->trajectory_.getDuration());
					cout << "poly start vel: " << endl;
					polyStartVel(0) = 1.0;
					Eigen::Vector3d polyEndVel (0.0, 0.0, 0.0);
					Eigen::Vector3d polyStartAcc = this->trajectory_.getDerivative().getDerivative().at(this->trajectory_.getDuration());
					Eigen::Vector3d polyEndAcc (0.0, 0.0, 0.0);
					polyStartEndCondition.push_back(polyStartVel);
					polyStartEndCondition.push_back(polyEndVel);
					polyStartEndCondition.push_back(polyStartAcc);
					polyStartEndCondition.push_back(polyEndAcc);
					cout << "update for pwl traj" << endl;
					this->polyTraj_->updatePath(waypoints, polyStartEndCondition);
					cout << "poly plan start" << endl;
					this->polyTraj_->makePlan(polyTrajTemp, false); // no corridor constraint
					cout << "poly plan success" << endl;
					for (geometry_msgs::PoseStamped ps : polyTrajTemp.poses){
						inputTraj.poses.push_back(ps);
					}
					this->polyTrajMsg_ = polyTrajTemp;
				}
			}
			this->inputTrajMsg_ = inputTraj;

			// start end conditions
			std::vector<Eigen::Vector3d> startEndCondition;
			this->getStartEndCondition(startEndCondition);
			cout << "start update" << endl;
			bool updateSuccess = this->bsplineTraj_->updatePath(inputTraj, startEndCondition);
			cout << "update sucess." << endl;
			if (updateSuccess){
				nav_msgs::Path bsplineTrajMsgTemp;
				bool planSuccess = this->bsplineTraj_->makePlan(bsplineTrajMsgTemp);
				if (planSuccess){
					this->bsplineTrajMsg_ = bsplineTrajMsgTemp;
					this->trajStartTime_ = ros::Time::now();
					this->trajectory_ = this->bsplineTraj_->getTrajectory();
					this->trajectoryReady_ = true;
					this->replan_ = false;
					cout << "[AutoFlight]: Trajectory generated successfully." << endl;
				}
				else{
					// if the current trajectory is still valid, then just ignore this iteration
					// if the current trajectory/or new goal point is assigned is not valid, then just stop
					if (this->hasCollision()){
						this->trajectoryReady_ = false;
						this->stop();
						cout << "[AutoFlight]: Stop!!! Trajectory generation fails." << endl;
					}
					else{
						cout << "[AutoFlight]: Trajectory fail. Use trajectory from previous iteration." << endl;
					}
				}
			}
		}
	}

	void navigation::replanCheckCB(const ros::TimerEvent&){
		/*
			Replan if
			1. collision detected
			2. new goal point assigned
			3. fixed distance
		*/
		if (this->goalReceived_){
			this->replan_ = false;
			this->trajectoryReady_ = false;
			if (not this->useYawControl_){
				double yaw = atan2(this->goal_.pose.position.y - this->odom_.pose.pose.position.y, this->goal_.pose.position.x - this->odom_.pose.pose.position.x);
				this->moveToOrientation(yaw, this->desiredAngularVel_);
			}
			this->replan_ = true;
			this->goalReceived_ = false;

			cout << "[AutoFlight]: Replan for new goal position." << endl; 
			return;
		}

		if (this->trajectoryReady_){
			if (this->hasCollision()){ // if trajectory not ready, do not replan
				this->replan_ = true;
				cout << "[AutoFlight]: Replan for collision." << endl;
				return;
			}


			if (this->computeExecutionDistance() >= 3.0){
				this->replan_ = true;
				cout << "[AutoFlight]: Regular replan." << endl;
				return;
			}


			// check whether reach the trajectory goal
			if (this->isReach(this->bsplineTrajMsg_.poses.back())){
				this->replan_  = true;
				this->trajectoryReady_ = false;
				return;
			}
		}
	}

	void navigation::trajExeCB(const ros::TimerEvent&){
		if (this->trajectoryReady_){
			ros::Time currTime = ros::Time::now();
			this->trajTime_ = (currTime - this->trajStartTime_).toSec();
			Eigen::Vector3d pos = this->trajectory_.at(this->trajTime_);
			Eigen::Vector3d vel = this->trajectory_.getDerivative().at(this->trajTime_);
			Eigen::Vector3d acc = this->trajectory_.getDerivative().getDerivative().at(this->trajTime_);
			
			tracking_controller::Target target;
			target.position.x = pos(0);
			target.position.y = pos(1);
			target.position.z = pos(2);
			target.velocity.x = vel(0);
			target.velocity.y = vel(1);
			target.velocity.z = vel(2);
			target.acceleration.x = acc(0);
			target.acceleration.y = acc(1);
			target.acceleration.z = acc(2);
			if (not this->useYawControl_){
				target.yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
			}
			else{
				target.yaw = atan2(vel(1), vel(0));
			}
			this->updateTargetWithState(target);			
		}
	}

	void navigation::stateUpdateCB(const ros::TimerEvent&){
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

	void navigation::visCB(const ros::TimerEvent&){
		if (this->rrtPathMsg_.poses.size() != 0){
			this->rrtPathPub_.publish(this->rrtPathMsg_);
		}
		if (this->polyTrajMsg_.poses.size() != 0){
			this->polyTrajPub_.publish(this->polyTrajMsg_);
		}
		if (this->pwlTrajMsg_.poses.size() != 0){
			this->pwlTrajPub_.publish(this->pwlTrajMsg_);
		}
		if (this->bsplineTrajMsg_.poses.size() != 0){
			this->bsplineTrajPub_.publish(this->bsplineTrajMsg_);
		}
		if (this->inputTrajMsg_.poses.size() != 0){
			this->inputTrajPub_.publish(this->inputTrajMsg_);
		}
	}

	void navigation::run(){
		// take off the drone
		this->takeoff();

		// register timer callback
		this->registerCallback();
	}

	void navigation::getStartEndCondition(std::vector<Eigen::Vector3d>& startEndCondition){
		/*	
			1. start velocity
			2. start acceleration (set to zero)
			3. end velocity
			4. end acceleration (set to zero) 
		*/

		Eigen::Vector3d currVel, currAcc;
		if (this->trajectoryReady_){
			currVel = this->currVel_;
			currAcc = this->currAcc_;
		}
		else{
			double targetYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
			currVel = this->desiredVel_ * Eigen::Vector3d (cos(targetYaw), sin(targetYaw), 0.0);
			currAcc = Eigen::Vector3d (0.0, 0.0, 0.0);			
		}
		Eigen::Vector3d endVel (0.0, 0.0, 0.0);
		Eigen::Vector3d endAcc (0.0, 0.0, 0.0);
		startEndCondition.push_back(currVel);
		startEndCondition.push_back(endVel);
		startEndCondition.push_back(currAcc);
		startEndCondition.push_back(endAcc);
	}

	bool navigation::hasCollision(){
		if (this->trajectoryReady_){
			for (double t=this->trajTime_; t<=this->trajectory_.getDuration(); t+=0.1){
				Eigen::Vector3d p = this->trajectory_.at(t);
				bool hasCollision = this->map_->isInflatedOccupied(p);
				if (hasCollision){
					return true;
				}
			}
		}
		return false;
	}

	double navigation::computeExecutionDistance(){
		if (this->trajectoryReady_){
			Eigen::Vector3d prevP, currP;
			bool firstTime = true;
			double totalDistance = 0.0;
			for (double t=0.0; t<=this->trajTime_; t+=0.1){
				currP = this->trajectory_.at(t);
				if (firstTime){
					firstTime = false;
				}
				else{
					totalDistance += (currP - prevP).norm();
				}
				prevP = currP;
			}
			return totalDistance;
		}
		return -1.0;
	}

	nav_msgs::Path navigation::getCurrentTraj(double dt){
		nav_msgs::Path currentTraj;
		currentTraj.header.frame_id = "map";
		currentTraj.header.stamp = ros::Time::now();
		if (this->trajectoryReady_){
			ros::Time currTime = ros::Time::now();
			double trajTime = (currTime - this->trajStartTime_).toSec();	
			for (double t=trajTime; t<=this->trajectory_.getDuration(); t+=dt){
				Eigen::Vector3d pos = this->trajectory_.at(t);
				geometry_msgs::PoseStamped ps;
				ps.pose.position.x = pos(0);
				ps.pose.position.y = pos(1);
				ps.pose.position.z = pos(2);
				currentTraj.poses.push_back(ps);
			}		
		}
		return currentTraj;
	}

}