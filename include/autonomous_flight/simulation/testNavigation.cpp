/*
	FILE: testNavigation.cpp
	------------------------
	test navigation implementation file in simulation
*/
#include <autonomous_flight/simulation/testNavigation.h>

namespace AutoFlight{
	testNavigation::testNavigation(const ros::NodeHandle& nh) : flightBase(nh){
		this->initModules();
		this->initParam();
		this->registerPub();
	}

	void testNavigation::initParam(){
		this->desiredVel_ = this->pwlTraj_->getDesiredVel();
	}

	void testNavigation::initModules(){
		// initialize map
		this->map_.reset(new mapManager::occMap (this->nh_));


		// initialize fake detector
		this->detector_.reset(new onboardVision::fakeDetector (this->nh_));

		// initialize piecewise linear trajectory planner
		this->pwlTraj_.reset(new trajPlanner::pwlTraj (this->nh_));

		// initialize bspline trajectory planner
		this->bsplineTraj_.reset(new trajPlanner::bsplineTraj (this->nh_));
		this->bsplineTraj_->setMap(this->map_);
	}

	void testNavigation::registerPub(){
		this->pwlTrajPub_ = this->nh_.advertise<nav_msgs::Path>("navigation/pwl_trajectory", 10);
		this->bsplineTrajPub_ = this->nh_.advertise<nav_msgs::Path>("navigation/bspline_trajectory", 10);
	}

	void testNavigation::registerCallback(){
		this->plannerTimer_ = this->nh_.createTimer(ros::Duration(0.1), &testNavigation::plannerCB, this);

		// trajectory execution timer
		// this->trajExeTimer_ = this->nh_.createTimer(ros::Duration(0.1), &testNavigation::trajExeCB, this);

		// visualization
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.1), &testNavigation::visCB, this);

		// free map timer
		this->freeMapTimer_ = this->nh_.createTimer(ros::Duration(0.01), &testNavigation::freeMapCB, this);


		// collision check
		this->collisionCheckTimer_ = this->nh_.createTimer(ros::Duration(0.05), &testNavigation::collisionCheckCB, this);
	}

	void testNavigation::run(){
		// take off the drone
		this->takeoff();

		// register timer callback
		this->registerCallback();
	}

	void testNavigation::plannerCB(const ros::TimerEvent&){
		if (not this->firstGoal_ or not this->goalReceived_) return;

		nav_msgs::Path simplePath;
		geometry_msgs::PoseStamped pStart, pGoal;
		pStart.pose = this->odom_.pose.pose;
		pGoal = this->goal_;
		std::vector<geometry_msgs::PoseStamped> pathVec {pStart, pGoal};
		simplePath.poses = pathVec;

		this->pwlTraj_->updatePath(simplePath);
		this->pwlTraj_->makePlan(this->pwlTrajMsg_, 0.1);


		// if (this->goalReceived_){
		// 	// move to the correct orientation
		// 	double yaw = atan2(pGoal.pose.position.y - pStart.pose.position.y, pGoal.pose.position.x - pStart.pose.position.x);
		// 	geometry_msgs::PoseStamped poseTgt;
		// 	poseTgt.pose.position = pStart.pose.position;
		// 	poseTgt.pose.orientation = AutoFlight::quaternion_from_rpy(0, 0, yaw);
		// 	nav_msgs::Path rotationPath, rotationTraj;
		// 	std::vector<geometry_msgs::PoseStamped> rotationPathVec {pStart, poseTgt};
		// 	rotationPath.poses = rotationPathVec;
		// 	this->pwlTraj_->updatePath(rotationPath, true);
		// 	this->pwlTraj_->makePlan(rotationTraj, 0.1);
		// 	this->td_.updateTrajectory(rotationTraj, this->pwlTraj_->getDuration());
	
		// 	ros::Rate r (50);
		// 	while (ros::ok() and not this->isReach(poseTgt)){
		// 		this->adjustingYaw_ = true;
		// 		ros::spinOnce();
		// 		r.sleep();
		// 	}
		// 	this->adjustingYaw_ = false;
		// 	this->goalReceived_ = false;
		// 	this->goalReceivedPWL_ = true;	
		// }


		std::vector<Eigen::Vector3d> startEndCondition;
		double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		Eigen::Vector3d startCondition (cos(currYaw), sin(currYaw), 0);
		startCondition *= this->desiredVel_;
		startEndCondition.push_back(startCondition); // start vel
		startEndCondition.push_back(Eigen::Vector3d (0, 0, 0)); //start acc condition
		startEndCondition.push_back(Eigen::Vector3d (0, 0, 0)); //end vel condition
		startEndCondition.push_back(Eigen::Vector3d (0, 0, 0)); //end acc condition

		bool updateSuccess = false;
		updateSuccess = this->bsplineTraj_->updatePath(this->pwlTrajMsg_, startEndCondition);


		if (updateSuccess){
			nav_msgs::Path bsplineTrajMsgTemp;
			bool planSuccess = this->bsplineTraj_->makePlan(bsplineTrajMsgTemp);
			if (planSuccess){
				this->bsplineTrajMsg_ = bsplineTrajMsgTemp;
				this->td_.updateTrajectory(this->bsplineTrajMsg_, this->bsplineTraj_->getDuration());
				this->trajValid_ = true;
			}
			else{
				this->td_.stop(this->odom_.pose.pose); // update stop trajectory
			}
		}
		this->goalReceived_ = false;
	}

	void testNavigation::trajExeCB(const ros::TimerEvent&){
		if (not this->td_.init){
			return;
		}
		this->updateTarget(this->td_.getPose());
	}

	void testNavigation::visCB(const ros::TimerEvent&){
		if (this->pwlTrajMsg_.poses.size() != 0){
			this->pwlTrajPub_.publish(this->pwlTrajMsg_);
		}
		if (this->bsplineTrajMsg_.poses.size() != 0){
			this->bsplineTrajPub_.publish(this->bsplineTrajMsg_);
		}
	}

	void testNavigation::freeMapCB(const ros::TimerEvent&){
		onboard_vision::ObstacleList msg;
		std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> freeRegions;
		this->detector_->getObstacles(msg);
		for (onboard_vision::Obstacle ob: msg.obstacles){
			Eigen::Vector3d lowerBound (ob.px-ob.xsize/2-0.3, ob.py-ob.ysize/2-0.3, ob.pz);
			Eigen::Vector3d upperBound (ob.px+ob.xsize/2+0.3, ob.py+ob.ysize/2+0.3, ob.pz+ob.zsize+0.2);
			freeRegions.push_back(std::make_pair(lowerBound, upperBound));
		}
		this->map_->updateFreeRegions(freeRegions);
	}

	void testNavigation::collisionCheckCB(const ros::TimerEvent&){
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

	void testNavigation::getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclesPos, std::vector<Eigen::Vector3d>& obstaclesVel, std::vector<Eigen::Vector3d>& obstaclesSize){
		onboard_vision::ObstacleList msg;
		this->detector_->getObstaclesInSensorRange(PI_const, msg);
		for (onboard_vision::Obstacle ob : msg.obstacles){
			Eigen::Vector3d pos (ob.px, ob.py, ob.pz);
			Eigen::Vector3d vel (ob.vx, ob.vy, ob.vz);
			Eigen::Vector3d size (ob.xsize, ob.ysize, ob.zsize);
			obstaclesPos.push_back(pos);
			obstaclesVel.push_back(vel);
			obstaclesSize.push_back(size);
		}
	}
}