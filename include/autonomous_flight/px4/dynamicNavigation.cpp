/*
	FILE: dynamicNavigation.cpp
	------------------------
	dynamic navigation implementation file in realworld
*/
#include <autonomous_flight/px4/dynamicNavigation.h>

namespace AutoFlight{
	dynamicNavigation::dynamicNavigation(const ros::NodeHandle& nh) : flightBase(nh){
		this->initModules();
		this->initParam();
		this->registerPub();
	}

	void dynamicNavigation::initParam(){
		this->desiredVel_ = this->pwlTraj_->getDesiredVel();
	}

	void dynamicNavigation::initModules(){
		// initialize map
		this->map_.reset(new mapManager::dynamicMap ());
		this->map_->initMap(this->nh_);


		// initialize piecewise linear trajectory planner
		this->pwlTraj_.reset(new trajPlanner::pwlTraj (this->nh_));

		// initialize bspline trajectory planner
		this->bsplineTraj_.reset(new trajPlanner::bsplineTraj (this->nh_));
		this->bsplineTraj_->setMap(this->map_);
	}

	void dynamicNavigation::registerPub(){
		this->pwlTrajPub_ = this->nh_.advertise<nav_msgs::Path>("navigation/pwl_trajectory", 10);
		this->bsplineTrajPub_ = this->nh_.advertise<nav_msgs::Path>("navigation/bspline_trajectory", 10);
		this->historyTrajPub_ = this->nh_.advertise<nav_msgs::Path>("navigation/history_traj", 10);
	}

	void dynamicNavigation::registerCallback(){
		// planner timer
		this->plannerTimer_ = this->nh_.createTimer(ros::Duration(0.5), &dynamicNavigation::plannerCB, this);
		
		// trajectory execution timer
		this->trajExeTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dynamicNavigation::trajExeCB, this);

		// history trajectory
		this->historyTrajTimer_ = this->nh_.createTimer(ros::Duration(0.2), &dynamicNavigation::historyTrajCB, this);

		// visualization
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dynamicNavigation::visCB, this);


		// collision check
		this->collisionCheckTimer_ = this->nh_.createTimer(ros::Duration(0.05), &dynamicNavigation::collisionCheckCB, this);
	}

	void dynamicNavigation::run(){
		// take off the drone
		this->takeoff();

		// register timer callback
		this->registerCallback();
	}

	void dynamicNavigation::adjustHeading(){
		this->adjustHeading_ = true;
		geometry_msgs::PoseStamped pStart, pGoal;
		pStart.pose = this->odom_.pose.pose;
		pGoal = this->goal_;
		double yaw = atan2(pGoal.pose.position.y - pStart.pose.position.y, pGoal.pose.position.x - pStart.pose.position.x);
		geometry_msgs::PoseStamped poseTgt;
		poseTgt.pose.position = pStart.pose.position;
		poseTgt.pose.orientation = AutoFlight::quaternion_from_rpy(0, 0, yaw);
		nav_msgs::Path rotationPath, rotationTraj;
		std::vector<geometry_msgs::PoseStamped> rotationPathVec {pStart, poseTgt};
		rotationPath.poses = rotationPathVec;
		this->pwlTraj_->updatePath(rotationPath, true);
		this->pwlTraj_->makePlan(rotationTraj, 0.1);
		this->td_.updateTrajectory(rotationTraj, this->pwlTraj_->getDuration());

		ros::Rate r (10);
		while (ros::ok() and not this->isReach(poseTgt)){
			ros::spinOnce();
			r.sleep();
		}
		this->adjustHeading_ = false;
	}

	void dynamicNavigation::getStartCondition(std::vector<Eigen::Vector3d>& startEndCondition){
		double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		Eigen::Vector3d startCondition (cos(currYaw), sin(currYaw), 0);
		startCondition *= this->desiredVel_;
		startEndCondition.push_back(startCondition); // start vel
		startEndCondition.push_back(Eigen::Vector3d (0, 0, 0)); //start acc condition
		startEndCondition.push_back(Eigen::Vector3d (0, 0, 0)); //end vel condition
		startEndCondition.push_back(Eigen::Vector3d (0, 0, 0)); //end acc condition
	}

	void dynamicNavigation::plannerCB(const ros::TimerEvent&){
		if (not this->firstGoal_) return;
		std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
		this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
		
		bool replan = this->goalReceived_ or (not this->trajValid_) or (obstaclesPos.size() != 0) or (this->td_.needReplan(1.0/3.0));
		
		if (this->goalReceived_){
			this->goal_.pose.position.z = this->takeoffHgt_;
			this->adjustHeading();
			this->goalReceived_ = false;
		}


		if (replan){
			// pwl
			nav_msgs::Path simplePath;
			geometry_msgs::PoseStamped pStart, pGoal;
			pStart.pose = this->odom_.pose.pose;
			pGoal = this->goal_;
			std::vector<geometry_msgs::PoseStamped> pathVec {pStart, pGoal};
			simplePath.poses = pathVec;
			this->pwlTraj_->updatePath(simplePath);
			this->pwlTraj_->makePlan(this->pwlTrajMsg_, 0.1);			

			// bspline
			std::vector<Eigen::Vector3d> startEndCondition;
			this->getStartCondition(startEndCondition);
			bool updateSuccess = false;
			updateSuccess = this->bsplineTraj_->updatePath(this->pwlTrajMsg_, startEndCondition);
			if (obstaclesPos.size() != 0){
				this->bsplineTraj_->updateDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}

			if (updateSuccess){
				nav_msgs::Path bsplineTrajMsgTemp;
				bool planSuccess = this->bsplineTraj_->makePlan(bsplineTrajMsgTemp);
				if (planSuccess){
					this->bsplineTrajMsg_ = bsplineTrajMsgTemp;
					this->td_.updateTrajectory(this->bsplineTrajMsg_, this->bsplineTraj_->getDuration());
					this->trajValid_ = true;
				}
				else{
					if (not this->trajValid_){
						this->td_.stop(this->odom_.pose.pose);
						cout << "[dynamicNavigation]: Failure stop!" << endl;
					}
					else{
						cout << "[dynamicNavigation]: Ignore this iteration!" << endl;
					}
				}
			}
		}

	}

	void dynamicNavigation::trajExeCB(const ros::TimerEvent&){
		if (not this->td_.init){
			return;
		}
		if (not adjustHeading_){
			this->updateTarget(this->td_.getPoseWithoutYaw(this->odom_.pose.pose));
		}
		else{
			this->updateTarget(this->td_.getPose(this->odom_.pose.pose));
		}
	}

	void dynamicNavigation::historyTrajCB(const ros::TimerEvent&){
		if (not firstGoal_) return;
		geometry_msgs::PoseStamped currPose;
		currPose.pose = this->odom_.pose.pose;
		this->historyTrajMsg_.header.frame_id = "map";
		this->historyTrajMsg_.poses.push_back(currPose);
	}

	void dynamicNavigation::visCB(const ros::TimerEvent&){
		if (this->pwlTrajMsg_.poses.size() != 0){
			this->pwlTrajPub_.publish(this->pwlTrajMsg_);
		}
		if (this->bsplineTrajMsg_.poses.size() != 0){
			this->bsplineTrajPub_.publish(this->bsplineTrajMsg_);
		}
		if (this->historyTrajMsg_.poses.size() != 0){
			this->historyTrajPub_.publish(this->historyTrajMsg_);
		}
	}


	void dynamicNavigation::collisionCheckCB(const ros::TimerEvent&){
		if (this->td_.currTrajectory.poses.size() == 0) return;
		nav_msgs::Path currTrajectory = this->td_.currTrajectory;
		for (geometry_msgs::PoseStamped ps : currTrajectory.poses){
			Eigen::Vector3d p (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
			if (this->map_->isInflatedOccupied(p)){
				this->trajValid_ = false;
				return;
			}
		}
		this->trajValid_ = true;
	}

}