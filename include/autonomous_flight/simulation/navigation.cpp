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
	}

	void navigation::registerCallback(){
		// rrt timer
		this->rrtTimer_ = this->nh_.createTimer(ros::Duration(0.5), &navigation::rrtCB, this);

		// poly traj timer
		this->polyTrajTimer_ = this->nh_.createTimer(ros::Duration(0.2), &navigation::polyTrajCB, this);

		// pwl timer
		this->pwlTimer_ = this->nh_.createTimer(ros::Duration(0.05), &navigation::pwlCB, this);
		
		// bspline timer
		this->bsplineTimer_ = this->nh_.createTimer(ros::Duration(0.05), &navigation::bsplineCB, this);

		// trajectory execution timer
		this->trajExeTimer_ = this->nh_.createTimer(ros::Duration(0.1), &navigation::trajExeCB, this);

		// visualization
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.1), &navigation::visCB, this);
	}

	void navigation::run(){
		// take off the drone
		this->takeoff();

		// register timer callback
		this->registerCallback();
	}

	void navigation::rrtCB(const ros::TimerEvent&){
		if (not this->firstGoal_) return;

		this->rrtPlanner_->updateStart(this->odom_.pose.pose);
		this->rrtPlanner_->updateGoal(this->goal_.pose);
		this->rrtPlanner_->makePlan(this->rrtPathMsg_);
		this->rrtPathUpdated_ = true;
	}

	void navigation::polyTrajCB(const ros::TimerEvent&){
		if (this->rrtPathMsg_.poses.size() == 0) return;
		if (this->rrtPathUpdated_ == false) return;

		this->polyTraj_->updatePath(this->rrtPathMsg_);
		this->polyTraj_->makePlan(this->polyTrajMsg_);
		this->rrtPathUpdated_ = false;
	}

	void navigation::pwlCB(const ros::TimerEvent&){
		if (not this->firstGoal_) return;

		nav_msgs::Path simplePath;
		geometry_msgs::PoseStamped pStart, pGoal;
		pStart.pose = this->odom_.pose.pose;
		pGoal = this->goal_;
		std::vector<geometry_msgs::PoseStamped> pathVec {pStart, pGoal};
		simplePath.poses = pathVec;

		this->pwlTraj_->updatePath(simplePath);
		this->pwlTraj_->makePlan(this->pwlTrajMsg_, 0.1);
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
				}
				else{
					ROS_INFO("Fail. Stop!");
					this->td_.stop(this->odom_.pose.pose);
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

	void navigation::visCB(const ros::TimerEvent&){
		this->rrtPathPub_.publish(this->rrtPathMsg_);
		this->polyTrajPub_.publish(this->polyTrajMsg_);
		this->pwlTrajPub_.publish(this->pwlTrajMsg_);
		this->bsplineTrajPub_.publish(this->bsplineTrajMsg_);
	}
}