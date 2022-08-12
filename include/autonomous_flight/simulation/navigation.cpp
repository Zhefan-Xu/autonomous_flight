/*
	FILE: navigation.cpp
	------------------------
	navigation implementation file in simulation
*/
#include <autonomous_flight/simulation/navigation.h>

namespace AutoFlight{
	navigation::navigation(const ros::NodeHandle& nh) : flightBase(nh){
		this->initModules();
		this->initParam();
		this->registerPub();
	}

	void navigation::initParam(){
		this->desiredVel_ = this->pwlTraj_->getDesiredVel();
	}

	void navigation::initModules(){
		// initialize map
		this->map_.reset(new mapManager::occMap (this->nh_));

		// initialize piecewise linear trajectory planner
		this->pwlTraj_.reset(new trajPlanner::pwlTraj (this->nh_));

		// initialize bspline trajectory planner
		this->bsplineTraj_.reset(new trajPlanner::bsplineTraj (this->nh_));
		this->bsplineTraj_->setMap(this->map_);
	}

	void navigation::registerPub(){
		this->pwlTrajPub_ = this->nh_.advertise<nav_msgs::Path>("navigation/pwl_trajectory", 10);
		this->bsplineTrajPub_ = this->nh_.advertise<nav_msgs::Path>("navigation/bspline_trajectory", 10);
	}

	void navigation::registerCallback(){
		// pwl timer
		this->pwlTimer_ = this->nh_.createTimer(ros::Duration(0.5), &navigation::pwlCB, this);
		
		// bspline timer
		this->bsplineTimer_ = this->nh_.createTimer(ros::Duration(0.05), &navigation::bsplineCB, this);

		// trajectory execution timer
		this->trajExeTimer_ = this->nh_.createTimer(ros::Duration(0.1), &navigation::trajExeCB, this);
	}

	void navigation::run(){
		// take off the drone
		this->takeoff();

		// register timer callback
		this->registerCallback();
	}

	void navigation::pwlCB(const ros::TimerEvent&){
		if (not this->goalReceived_ and not this->needNewPwlTraj_){
			return; 
		}

		if (this->goalReceived_){
			this->stopBsplinePlan_ = false;
			this->restartNum_ = 0;
		}
		nav_msgs::Path simplePath;
		geometry_msgs::PoseStamped pStart, pGoal;
		pStart.pose = this->odom_.pose.pose;
		pGoal = this->goal_;
		std::vector<geometry_msgs::PoseStamped> pathVec {pStart, pGoal};
		simplePath.poses = pathVec;

		this->pwlTraj_->updatePath(simplePath);
		this->pwlTraj_->makePlan(this->pwlTrajMsg_, 0.1);
		this->pwlTrajPub_.publish(this->pwlTrajMsg_);
		this->pwlTrajUpdated_ = true;
		this->goalReceived_ = false;
		this->needNewPwlTraj_ = false;
	}

	void navigation::bsplineCB(const ros::TimerEvent&){
		// only update if get new reference trajectory or current trajectory is not valid
		Eigen::Vector3d firstCollisionPos;
		if (not this->pwlTrajUpdated_ and this->bsplineTraj_->isCurrTrajValid(firstCollisionPos)){
			return;
		}
		if (this->stopBsplinePlan_){
			return;
		}

		std::vector<Eigen::Vector3d> startEndCondition;
		double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		Eigen::Vector3d startCondition (cos(currYaw), sin(currYaw), 0);
		startCondition *= this->desiredVel_;
		startEndCondition.push_back(startCondition); // start vel
		startEndCondition.push_back(Eigen::Vector3d (0, 0, 0)); //start acc condition
		startEndCondition.push_back(Eigen::Vector3d (0, 0, 0)); //end vel condition
		startEndCondition.push_back(Eigen::Vector3d (0, 0, 0)); //end acc condition

		bool updateSuccess = false;
		if (this->pwlTrajUpdated_){
			updateSuccess = this->bsplineTraj_->updatePath(this->pwlTrajMsg_, startEndCondition);
			this->pwlTrajUpdated_ = false;
		}
		else{
			updateSuccess = this->bsplineTraj_->updatePath(this->td_.currTrajectory, startEndCondition);
		}

		if (updateSuccess){
			bool planSuccess = this->bsplineTraj_->makePlan(this->bsplineTrajMsg_);
			if (not planSuccess and not this->bsplineTraj_->isCurrTrajValid()){
				Eigen::Vector3d currPos (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
				double collisionDist = (currPos - firstCollisionPos).norm();
				if (collisionDist <= 2.5){
					this->td_.backoff(this->odom_.pose.pose);
					ROS_WARN("BACKOFF TRIGGER!!!");
				}
				this->bsplineTrajPub_.publish(this->bsplineTrajMsg_);
				this->needNewPwlTraj_ = true;
				++this->restartNum_;
				if (this->restartNum_ >= 3){
					this->stopBsplinePlan_ = true;
					ROS_WARN("Impossible to find path. Stop trying.");
					return;
				}
				ROS_WARN("Cannot find valid path! Ignore this iteration. Restart next time.");
				return;
			}

			this->td_.updateTrajectory(this->bsplineTrajMsg_, this->bsplineTraj_->getDuration());
			this->bsplineTrajPub_.publish(this->bsplineTrajMsg_);
		}
	}

	void navigation::trajExeCB(const ros::TimerEvent&){
		if (not this->td_.init){
			return;
		}
		this->updateTarget(this->td_.getPose());
	}
}