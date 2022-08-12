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
		this->pwlTimer_ = this->nh_.createTimer(ros::Duration(0.05), &navigation::pwlCB, this);
		
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
		if (not this->firstGoal_){
			return; 
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
	}

	void navigation::bsplineCB(const ros::TimerEvent&){
		if (this->pwlTrajMsg_.poses.size() == 0) return;
		// update when current trajectory is not valid or new goal received
		Eigen::Vector3d firstCollisionPos;
		bool trajValid = this->bsplineTraj_->isCurrTrajValid(firstCollisionPos);

		if (this->goalReceived_ or not trajValid){
			this->goalReceived_ = false;
			std::vector<Eigen::Vector3d> startEndCondition;
			double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
			Eigen::Vector3d startCondition (cos(currYaw), sin(currYaw), 0);
			startCondition *= this->desiredVel_;
			startEndCondition.push_back(startCondition); // start vel
			startEndCondition.push_back(Eigen::Vector3d (0, 0, 0)); //start acc condition
			startEndCondition.push_back(Eigen::Vector3d (0, 0, 0)); //end vel condition
			startEndCondition.push_back(Eigen::Vector3d (0, 0, 0)); //end acc condition

			bool updateSuccess = this->bsplineTraj_->updatePath(this->pwlTrajMsg_, startEndCondition);


			if (updateSuccess){
				bool planSuccess = this->bsplineTraj_->makePlan(this->bsplineTrajMsg_);
				if (planSuccess){
					this->td_.updateTrajectory(this->bsplineTrajMsg_, this->bsplineTraj_->getDuration());
					this->bsplineTrajPub_.publish(this->bsplineTrajMsg_);
				}
			}
		}
	}

	void navigation::trajExeCB(const ros::TimerEvent&){
		if (not this->td_.init){
			return;
		}
		this->updateTarget(this->td_.getPose());
	}
}