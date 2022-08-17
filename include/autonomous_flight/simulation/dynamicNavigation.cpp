/*
	FILE: dynamicNavigation.cpp
	------------------------
	dynamic navigation implementation file in simulation
*/
#include <autonomous_flight/simulation/dynamicNavigation.h>

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
		this->map_.reset(new mapManager::occMap (this->nh_));

		// initialize fake detector
		this->detector_.reset(new onboardVision::fakeDetector (this->nh_));

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

	void dynamicNavigation::registerPub(){
		this->rrtPathPub_ = this->nh_.advertise<nav_msgs::Path>("navigation/rrt_path", 10);
		this->polyTrajPub_ = this->nh_.advertise<nav_msgs::Path>("navigation/poly_traj", 10);
		this->pwlTrajPub_ = this->nh_.advertise<nav_msgs::Path>("navigation/pwl_trajectory", 10);
		this->bsplineTrajPub_ = this->nh_.advertise<nav_msgs::Path>("navigation/bspline_trajectory", 10);
	}

	void dynamicNavigation::registerCallback(){
		// rrt timer
		this->rrtTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dynamicNavigation::rrtCB, this);

		// poly traj timer
		this->polyTrajTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dynamicNavigation::polyTrajCB, this);

		// pwl timer
		this->pwlTimer_ = this->nh_.createTimer(ros::Duration(0.05), &dynamicNavigation::pwlCB, this);
		
		// bspline timer
		this->bsplineTimer_ = this->nh_.createTimer(ros::Duration(0.05), &dynamicNavigation::bsplineCB, this);

		// trajectory execution timer
		this->trajExeTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dynamicNavigation::trajExeCB, this);

		// visualization
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dynamicNavigation::visCB, this);

		// free map timer
		this->freeMapTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dynamicNavigation::freeMapCB, this);
	}

	void dynamicNavigation::run(){
		// take off the drone
		this->takeoff();

		// register timer callback
		this->registerCallback();
	}

	void dynamicNavigation::rrtCB(const ros::TimerEvent&){
		if (not this->bsplineFailure_) return;
		if (not this->firstGoal_) return;
		// if (this->rrtPlanner_->isCurrPathValid() and not this->rrtPlanner_->hasNewGoal(this->goal_.pose)) return;

		this->rrtPlanner_->updateStart(this->odom_.pose.pose);
		this->rrtPlanner_->updateGoal(this->goal_.pose);
		this->rrtPlanner_->makePlan(this->rrtPathMsg_);
		this->rrtPathUpdated_ = true;
	}

	void dynamicNavigation::polyTrajCB(const ros::TimerEvent&){
		if (this->rrtPathMsg_.poses.size() == 0) return;
		if (this->rrtPathUpdated_ == false) return;

		this->polyTraj_->updatePath(this->rrtPathMsg_);
		geometry_msgs::Twist vel;
		double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		Eigen::Vector3d startCondition (cos(currYaw), sin(currYaw), 0);
		startCondition *= this->desiredVel_;
		vel.linear.x = startCondition(0);  vel.linear.y = startCondition(1);  vel.linear.z = startCondition(2); 
		this->polyTraj_->updateInitVel(vel);
		this->polyTraj_->makePlan(this->polyTrajMsg_);
		this->rrtPathUpdated_ = false;
		this->useGlobalTraj_ = true;
		this->bsplineFailure_ = false; // reset bspline failure
		ROS_INFO("Global Trajectory Generated!");
	}

	void dynamicNavigation::pwlCB(const ros::TimerEvent&){
		if (not this->firstGoal_) return;

		nav_msgs::Path simplePath;
		geometry_msgs::PoseStamped pStart, pGoal;
		pStart.pose = this->odom_.pose.pose;
		pGoal = this->goal_;
		std::vector<geometry_msgs::PoseStamped> pathVec {pStart, pGoal};
		simplePath.poses = pathVec;

		this->pwlTraj_->updatePath(simplePath);
		this->pwlTraj_->makePlan(this->pwlTrajMsg_, 0.1);

		if (this->goalReceived_){
			this->goalReceived_ = false;
			this->goalReceivedPWL_ = true;
		}
	}

	void dynamicNavigation::bsplineCB(const ros::TimerEvent&){
		if (this->pwlTrajMsg_.poses.size() == 0) return;
		// update when current trajectory is not valid or new goal received
		Eigen::Vector3d firstCollisionPos;
		bool trajValid = this->bsplineTraj_->isCurrTrajValid(firstCollisionPos);


		if (this->goalReceivedPWL_ or not trajValid){
			std::vector<Eigen::Vector3d> startEndCondition;
			double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
			Eigen::Vector3d startCondition (cos(currYaw), sin(currYaw), 0);
			startCondition *= this->desiredVel_;
			startEndCondition.push_back(startCondition); // start vel
			startEndCondition.push_back(Eigen::Vector3d (0, 0, 0)); //start acc condition
			startEndCondition.push_back(Eigen::Vector3d (0, 0, 0)); //end vel condition
			startEndCondition.push_back(Eigen::Vector3d (0, 0, 0)); //end acc condition

			bool updateSuccess = false;
			if (not this->useGlobalTraj_ and not this->bsplineFailure_){
				updateSuccess = this->bsplineTraj_->updatePath(this->pwlTrajMsg_, startEndCondition);
			}
			
			if (this->useGlobalTraj_){
				updateSuccess = this->bsplineTraj_->updatePath(this->polyTrajMsg_, startEndCondition);
			}

			if (this->goalReceivedPWL_){
				this->goalReceivedPWL_ = false;
			}


			if (updateSuccess){
				nav_msgs::Path bsplineTrajMsgTemp;
				bool planSuccess = this->bsplineTraj_->makePlan(bsplineTrajMsgTemp);
				if (planSuccess){
					this->bsplineTrajMsg_ = bsplineTrajMsgTemp;
					this->td_.updateTrajectory(this->bsplineTrajMsg_, this->bsplineTraj_->getDuration());
				}
				else{
					if (this->useGlobalTraj_){
						ROS_INFO("Environment is not explored enough. Impossible to find a feasible trajectory!!! Please change your goal...");	
					}
					else{
						// if the goal is not valid. Do not replan
						if (this->map_->isInflatedOccupied(Eigen::Vector3d (this->goal_.pose.position.x, this->goal_.pose.position.y, this->goal_.pose.position.z))){
							ROS_INFO("Your selected goal is not collision-free. Please change your goal.");
						}
						else{
							this->bsplineFailure_ = true;
							ROS_INFO("Bspline failure. Trying replan with global path!");
						}
					}
					this->td_.stop(this->odom_.pose.pose);
				}
			}

			if (this->useGlobalTraj_){
				this->useGlobalTraj_ = false;
			}
		}
	}

	void dynamicNavigation::trajExeCB(const ros::TimerEvent&){
		if (not this->td_.init){
			return;
		}
		this->updateTarget(this->td_.getPose());
	}

	void dynamicNavigation::visCB(const ros::TimerEvent&){
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
	}

	void dynamicNavigation::freeMapCB(const ros::TimerEvent&){
		onboard_vision::ObstacleList msg;
		std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> freeRegions;
		this->detector_->getObstacles(msg);
		for (onboard_vision::Obstacle ob: msg.obstacles){
			Eigen::Vector3d lowerBound (ob.px-ob.xsize/2-0.3, ob.py-ob.ysize/2-0.3, ob.pz-0.4);
			Eigen::Vector3d upperBound (ob.px+ob.xsize/2+0.3, ob.py+ob.ysize/2+0.3, ob.pz+ob.zsize+0.3);
			freeRegions.push_back(std::make_pair(lowerBound, upperBound));
		}
		this->map_->updateFreeRegions(freeRegions);
	}
}