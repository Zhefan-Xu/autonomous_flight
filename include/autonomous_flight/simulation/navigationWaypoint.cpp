/*
	FILE: navigationWaypoint.cpp
	------------------------
	navigation through waypoint implementation file in simulation
*/
#include <autonomous_flight/simulation/navigationWaypoint.h>

namespace AutoFlight{
	navigationWaypoint::navigationWaypoint(const ros::NodeHandle& nh) : flightBase(nh){
		this->initModules();
		this->initParam();
		this->registerPub();
	}

	void navigationWaypoint::initParam(){
		this->desiredVel_ = this->pwlTraj_->getDesiredVel();
	}

	void navigationWaypoint::initModules(){
		// initialize map
		this->map_.reset(new mapManager::occMap (this->nh_));
		// this->map_.reset(new mapManager::dynamicMap ());
		// this->map_->initMap(this->nh_);

		// initialize polynomial trajectory planner
		this->polyTraj_.reset(new trajPlanner::polyTrajOccMap (this->nh_));
		this->polyTraj_->setMap(this->map_);

		// initialize piecewise linear trajectory planner
		this->pwlTraj_.reset(new trajPlanner::pwlTraj (this->nh_));

		// initialize bspline trajectory planner
		this->bsplineTraj_.reset(new trajPlanner::bsplineTraj (this->nh_));
		this->bsplineTraj_->setMap(this->map_);
	}

	void navigationWaypoint::registerPub(){
		this->polyTrajPub_ = this->nh_.advertise<nav_msgs::Path>("navigation/poly_traj", 10);
		this->pwlTrajPub_ = this->nh_.advertise<nav_msgs::Path>("navigation/pwl_trajectory", 10);
		this->bsplineTrajPub_ = this->nh_.advertise<nav_msgs::Path>("navigation/bspline_trajectory", 10);
	}

	void navigationWaypoint::registerCallback(){
		// planner timer
		this->plannerTimer_ = this->nh_.createTimer(ros::Duration(0.1), &navigationWaypoint::plannerCB, this);
		
		// initialize fake detector
		this->detector_.reset(new onboardVision::fakeDetector (this->nh_));
		// trajectory execution timer
		this->trajExeTimer_ = this->nh_.createTimer(ros::Duration(0.1), &navigationWaypoint::trajExeCB, this);

		// visualization
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.01), &navigationWaypoint::visCB, this);

		this->freeMapTimer_ = this->nh_.createTimer(ros::Duration(0.01), &navigationWaypoint::freeMapCB, this);

		// collision check
		this->collisionCheckTimer_ = this->nh_.createTimer(ros::Duration(0.05), &navigationWaypoint::collisionCheckCB, this);
	}

	void navigationWaypoint::updateWaypoints(const nav_msgs::Path& waypoints){
		this->waypoints_ = waypoints;
		geometry_msgs::PoseStamped pStart, pGoal;
		pStart.pose = this->odom_.pose.pose;
		pGoal = waypoints.poses[0];


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

		ros::Rate r (50);
		while (ros::ok() and not this->isReach(poseTgt)){
			ros::spinOnce();
			r.sleep();
		}
		this->init_ = true;
	}

	void navigationWaypoint::generatePWLTraj(nav_msgs::Path& pwlTraj){
		// check the nearest waypoint if too close then go next
		nav_msgs::Path waypoints;
		this->getCurrWaypoints(waypoints);

		this->pwlTraj_->updatePath(waypoints);
		this->pwlTraj_->makePlan(pwlTraj, 0.1);
	}

	void navigationWaypoint::getCurrWaypoints(nav_msgs::Path& waypoints){
		geometry_msgs::PoseStamped pCurr, nextWaypoint;
		pCurr.pose = this->odom_.pose.pose;
		nextWaypoint = this->waypoints_.poses[this->nextWaypointIdx_];

		// double dist = pow(pow(pCurr.pose.position.x - nextWaypoint.pose.position.x, 2) + pow(pCurr.pose.position.y - nextWaypoint.pose.position.y, 2) + pow(pCurr.pose.position.z - nextWaypoint.pose.position.z, 2), 0.5);
		// if (dist <= 1.0){
		if (this->isReach(nextWaypoint, false)){
			if (this->nextWaypointIdx_ != int(this->waypoints_.poses.size()-1)){
				++this->nextWaypointIdx_;	
			}
			geometry_msgs::PoseStamped pStart, pGoal;
			pStart.pose = this->odom_.pose.pose;
			pGoal = this->waypoints_.poses[this->nextWaypointIdx_];


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

			ros::Rate r (50);
			while (ros::ok() and not this->isReach(poseTgt)){
				ros::spinOnce();
				r.sleep();
			}
		} 

		waypoints.poses.push_back(pCurr);
		geometry_msgs::PoseStamped pNext = this->waypoints_.poses[this->nextWaypointIdx_];
		waypoints.poses.push_back(pNext);

		// for (size_t i=this->nextWaypointIdx_; i<this->waypoints_.poses.size(); ++i){
		// 	waypoints.poses.push_back(this->waypoints_.poses[i]);
		// }
	}

	void navigationWaypoint::run(const nav_msgs::Path& waypoints){
		// take off the drone
		this->takeoff();

		// register timer callback
		this->registerCallback();

		// update waypoints
		this->updateWaypoints(waypoints);
	}

	void navigationWaypoint::getStartEndConditions(std::vector<Eigen::Vector3d>& startEndCondition){
		double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		Eigen::Vector3d startCondition (cos(currYaw), sin(currYaw), 0);
		startCondition *= this->desiredVel_;
		startEndCondition.push_back(startCondition); // start vel
		startEndCondition.push_back(Eigen::Vector3d (0, 0, 0)); //start acc condition


		Eigen::Vector3d endCondition (0, 0, 0);
		// if (this->nextWaypointIdx_ != int(this->waypoints_.poses.size()-1)){
		// 	geometry_msgs::PoseStamped pNext = this->waypoints_.poses[this->nextWaypointIdx_];
		// 	geometry_msgs::PoseStamped pNextNext = this->waypoints_.poses[this->nextWaypointIdx_+1];
		// 	double yaw = atan2(pNextNext.pose.position.y - pNext.pose.position.y, pNextNext.pose.position.x - pNext.pose.position.x);
		// 	endCondition(0) = cos(yaw);
		// 	endCondition(1) = sin(yaw);
		// 	endCondition *= this->desiredVel_;
		// }
		startEndCondition.push_back(endCondition); //end vel condition
		startEndCondition.push_back(Eigen::Vector3d (0, 0, 0)); //end acc condition
	}

	void navigationWaypoint::plannerCB(const ros::TimerEvent&){
		if (not this->init_) return;
		this->generatePWLTraj(this->pwlTrajMsg_);

		std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
		this->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);

		bool planForDynamicObstacle = false;
		if (obstaclesPos.size() != 0){
			planForDynamicObstacle = true;
		}
		if (not this->trajValid_ or (this->td_.needReplan(1.0/3.0)) or planForDynamicObstacle){
			std::vector<Eigen::Vector3d> startEndCondition;
			this->getStartEndConditions(startEndCondition);

			
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
					this->td_.stop(this->odom_.pose.pose);
				}
			}
		}
	}

	void navigationWaypoint::trajExeCB(const ros::TimerEvent&){
		if (not this->td_.init){
			return;
		}

		this->updateTarget(this->td_.getPose());
	}

	void navigationWaypoint::visCB(const ros::TimerEvent&){
		if (this->polyTrajMsg_.poses.size() != 0){
			this->polyTrajPub_.publish(this->polyTrajMsg_);
		}
		if (this->pwlTrajMsg_.poses.size() != 0){
			this->pwlTrajPub_.publish(this->pwlTrajMsg_);
		}
		if (this->bsplineTrajMsg_.poses.size() != 0){
			this->bsplineTrajPub_.publish(this->bsplineTrajMsg_);
		}
	}

	void navigationWaypoint::freeMapCB(const ros::TimerEvent&){
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

	void navigationWaypoint::collisionCheckCB(const ros::TimerEvent&){
		if (this->td_.currTrajectory.poses.size() == 0) return;
		nav_msgs::Path currTrajectory = this->td_.currTrajectory;
		std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
		this->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
		// this->map_->getObjPos(obstaclesPos);
		// this->map_->getObjVel(obstaclesVel);
		// this->map_->getObjSize(obstaclesSize);
		Eigen::Vector3d obstaclePos, obstacleSize, diff;
		double size, dist;

		// this->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
		for (geometry_msgs::PoseStamped ps : currTrajectory.poses){
			Eigen::Vector3d p (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
			if (this->map_->isInflatedOccupied(p)){
				this->trajValid_ = false;
				return;
			}

			for (size_t i=0; i<obstaclesPos.size(); ++i){
				obstaclePos = obstaclesPos[i];
				obstacleSize = obstaclesSize[i];
				size = std::min(obstacleSize(0)/2, obstacleSize(1)/2);
				diff = p - obstaclePos;
				diff(2) = 0.0;
				dist = diff.norm() - size;
				if (dist < 0){
					this->trajValid_ = false;	
					return;
				}
			}

		}
		this->trajValid_ = true;
	}

	void navigationWaypoint::getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclesPos, std::vector<Eigen::Vector3d>& obstaclesVel, std::vector<Eigen::Vector3d>& obstaclesSize){
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