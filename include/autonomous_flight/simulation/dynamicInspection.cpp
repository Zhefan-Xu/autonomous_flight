/*
	FILE: dynamicInspection.cpp
	-----------------------------
	Implementation of dynamic inspection
*/

#include <autonomous_flight/simulation/dynamicInspection.h>

namespace AutoFlight{
	dynamicInspection::dynamicInspection(const ros::NodeHandle& nh) : flightBase(nh){
		this->initModules();
		this->initParam();
		this->registerPub();
	}

	void dynamicInspection::initParam(){
		this->desiredVel_ = this->pwlTraj_->getDesiredVel();
		this->desiredAngularVel_ = this->pwlTraj_->getDesiredAngularVel();

		// inspection moving velocity
		if (not this->nh_.getParam("inspection_velocity", this->inspectionVel_)){
			this->inspectionVel_ = 0.5;
			cout << "[AutoFlight]: No inspection velocity param. Use default 0.5m/s." << endl;
		}
		else{
			cout << "[AutoFlight]: Inspection velocity is set to: " << this->inspectionVel_ << "m/s." << endl;
		}	

		// minimum wall area
		if (not this->nh_.getParam("min_wall_area", this->minWallArea_)){
			this->minWallArea_ = 20;
			cout << "[AutoFlight]: No minimum wall area param. Use default 20m^2." << endl;
		}
		else{
			cout << "[AutoFlight]: Minimum wall area is set to: " << this->minWallArea_ << "m^2." << endl;
		}	

		// safe distance to all walls
		if (not this->nh_.getParam("safe_distance_to_wall", this->safeDistance_)){
			this->safeDistance_ = 2.5;
			cout << "[AutoFlight]: No safe distance to wall param. Use default 2.5m." << endl;
		}
		else{
			cout << "[AutoFlight]: Safe distance to wall is set to: " << this->safeDistance_ << "m." << endl;
		}	

		// side safe distance to all walls
		if (not this->nh_.getParam("side_safe_distance", this->sideSafeDistance_)){
			this->sideSafeDistance_ = 1.5;
			cout << "[AutoFlight]: No side safe distance param. Use default 1.5m." << endl;
		}
		else{
			cout << "[AutoFlight]: Side safe distance is set to: " << this->sideSafeDistance_ << "m." << endl;
		}		

		// inspection height
		if (not this->nh_.getParam("inspection_height", this->inspectionHeight_)){
			this->inspectionHeight_ = 2.5;
			cout << "[AutoFlight]: No inspection height param. Use default 2.5m." << endl;
		}
		else{
			cout << "[AutoFlight]: Inspection height is set to: " << this->inspectionHeight_ << "m." << endl;
		}		

		// ascend step
		if (not this->nh_.getParam("ascend_step", this->ascendStep_)){
			this->ascendStep_ = 3.0;
			cout << "[AutoFlight]: No ascend step param. Use default 3.0m." << endl;
		}
		else{
			cout << "[AutoFlight]: Ascend step is set to: " << this->ascendStep_ << "m." << endl;
		}	

		// descend step
		if (not this->nh_.getParam("descend_step", this->descendStep_)){
			this->descendStep_ = 2.0;
			cout << "[AutoFlight]: No descend step param. Use default 2.0m." << endl;
		}
		else{
			cout << "[AutoFlight]: Descend step is set to: " << this->descendStep_ << "m." << endl;
		}	

		// sensor range
		if (not this->nh_.getParam("sensor_range", this->sensorRange_)){
			this->sensorRange_ = 2.0;
			cout << "[AutoFlight]: No sensor range param. Use default 2.0m." << endl;
		}
		else{
			cout << "[AutoFlight]: Sensor range is set to: " << this->sensorRange_ << "m." << endl;
		}	

		// horizontal sensor angle 
		if (not this->nh_.getParam("sensor_angle_horizontal", this->sensorAngleH_)){
			this->sensorAngleH_ = PI_const/2;
			cout << "[AutoFlight]: No horizontal sensor angle param. Use default 90 degree." << endl;
		}
		else{
			cout << "[AutoFlight]: Horizontal sensor angle is set to: " << this->sensorAngleH_ << " degree." << endl;
			this->sensorAngleH_ *= PI_const/180.0;
		}	

		// vertical sensor angle 
		if (not this->nh_.getParam("sensor_angle_vertical", this->sensorAngleV_)){
			this->sensorAngleV_ = PI_const/3;
			cout << "[AutoFlight]: No vertical sensor angle param. Use default 60 degree." << endl;
		}
		else{
			cout << "[AutoFlight]: Vertical sensor angle is set to: " << this->sensorAngleV_ << " degree." << endl;
			this->sensorAngleV_ *= PI_const/180.0;
		}	

		// sample number
		if (not this->nh_.getParam("explore_sample_number", this->exploreSampleNum_)){
			this->exploreSampleNum_ = 10;
			cout << "[AutoFlight]: No exploration sample number param. Use default: 10." << endl;
		}
		else{
			cout << "[AutoFlight]: Exploration sample number is set to: " << this->exploreSampleNum_ << endl;
		}	

		// inspection goal given or not
		if (not this->nh_.getParam("inspection_goal_given", this->inspectionGoalGiven_)){
			this->inspectionGoalGiven_ = false;
			cout << "[AutoFlight]: Use default mode" << endl;
		}
		else{
			cout << "[AutoFlight]: use inspection goal is set to: " << this->inspectionGoalGiven_ << endl;
		}

		// inspection width given or not
		if (not this->nh_.getParam("inspection_width_given", this->inspectionWidthGiven_)){
			this->inspectionWidthGiven_ = false;
			cout << "[AutoFlight]: Use default mode" << endl;
		}
		else{
			cout << "[AutoFlight]: use inspection width is set to: " << this->inspectionWidthGiven_ << endl;
		}

		if (this->inspectionGoalGiven_){
			// inspection location (last component is orientation in degree)
			std::vector<double> inspectionGoalTemp;
			if (not this->nh_.getParam("inspection_goal", inspectionGoalTemp)){
				this->inspectionGoalGiven_ = false;
				cout << "[AutoFlight]: Use default inspection mode." << endl;
			}
			else{
				this->inspectionGoal_(0) = inspectionGoalTemp[0];
				this->inspectionGoal_(1) = inspectionGoalTemp[1];
				this->inspectionGoal_(2) = inspectionGoalTemp[2];
				this->inspectionOrientation_ = inspectionGoalTemp[3];
				cout << "[AutoFlight]: Inspection goal is set to: [" << this->inspectionGoal_(0)  << ", " << this->inspectionGoal_(1) << ", " <<  this->inspectionGoal_(2) << "]." << endl; 
				cout << "[AutoFlight]: Inspection angle is set to: " << this->inspectionOrientation_ << " degree." << endl;
				this->inspectionOrientation_ *= PI_const/180;
			}
		}

		if (this->inspectionWidthGiven_){
			// inspection width
			if (not this->nh_.getParam("inspection_width", this->inspectionWidth_)){
				this->inspectionWidthGiven_ = false;
				cout << "[AutoFlight]: No inspection width parameter." << endl;
			}
			else{
				this->inspectionWidthGiven_ = true;
				cout << "[AutoFlight]: Inspection width is set to: " << this->inspectionWidth_ << "m." << endl;
			}
		}

		// side wall raycast range
		if (not this->nh_.getParam("side_wall_raycast_range", this->sideWallRaycastRange_)){
			this->sideWallRaycastRange_ = 5.0;
			cout << "[AutoFlight]: No side wall raycast range parameter. Use default: 5.0m" << endl;
		}
		else{
			cout << "[AutoFlight]: Side wall raycast range is set to: " << this->sideWallRaycastRange_ << "m." << endl;
		}

		// side wall minimum length threshold
		if (not this->nh_.getParam("side_wall_min_length", this->sideWallLengthThresh_)){
			this->sideWallLengthThresh_ = 5.0;
			cout << "[AutoFlight]: No side wall min length parameter. Use default: 5.0m" << endl;
		}
		else{
			cout << "[AutoFlight]: Side wall min length is set to: " << this->sideWallLengthThresh_ << "m." << endl;
		}		

		// side wall max angle threshold
		if (not this->nh_.getParam("side_wall_max_angle", this->sideWallAngleThresh_)){
			this->sideWallAngleThresh_ = PI_const/2;
			cout << "[AutoFlight]: No side wall max angle parameter. Use default: 90 degree." << endl;
		}
		else{
			cout << "[AutoFlight]: Side wall max angle is set to: " << this->sideWallAngleThresh_ << " degree." << endl;
			this->sideWallAngleThresh_ *= PI_const/180.0;
		}	
	}

	void dynamicInspection::initModules(){
		// initialize map
		this->map_.reset(new mapManager::dynamicMap (this->nh_));

		// initialize fake detector
		// this->detector_.reset(new onboardVision::fakeDetector (this->nh_));

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

	void dynamicInspection::registerPub(){
		// goal publisher
		this->goalPub_ = this->nh_.advertise<geometry_msgs::PoseStamped>("inspection/current_goal", 10);

		// rrt path publisher
		this->rrtPathPub_ = this->nh_.advertise<nav_msgs::Path>("inspection/rrt_path", 10);

		// polynomial trajectory publisher
		this->polyTrajPub_ = this->nh_.advertise<nav_msgs::Path>("inspection/poly_trajectory", 10);

		// piecewise linear trajectory publisher
		this->pwlTrajPub_ = this->nh_.advertise<nav_msgs::Path>("inspection/pwl_trajectory", 10);

		// bspline trajectory publisher
		this->bsplineTrajPub_ = this->nh_.advertise<nav_msgs::Path>("inspection/bspline_trajectory", 10);

		// wall visualization publisher
		this->wallVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("inspection/wall", 10);

		// side wall visualzation publisher
		image_transport::ImageTransport it (this->nh_);
		this->sideWallRaycastVisPub_ = it.advertise("inspection/side_wall_raycast", 1);
	}

	void dynamicInspection::registerCallback(){
		// planner callback
		this->plannerTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dynamicInspection::plannerCB, this);

		// trajectory execution callback
		this->trajExeTimer_ = this->nh_.createTimer(ros::Duration(0.02), &dynamicInspection::trajExeCB, this);

		// check wall callback
		if (not this->inspectionGoalGiven_){
			this->checkWallTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dynamicInspection::checkWallCB, this);
		}	

		// collision check callback
		this->collisionCheckTimer_ = this->nh_.createTimer(ros::Duration(0.02), &dynamicInspection::collisionCheckCB, this);

		// side wall raycast callback
		this->sideWallRaycastTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dynamicInspection::sideWallRaycastCB, this);

		// visualization callback
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.03), &dynamicInspection::visCB, this);
	}

	void dynamicInspection::run(){
		this->takeoff();

		this->registerCallback();
	}

	void dynamicInspection::plannerCB(const ros::TimerEvent&){
		if (this->flightState_ == FLIGHT_STATE::FORWARD){
			this->useYaw_ = true;
			// navigate to the goal position			
			// first try with pwl trajectory if not work try with the global path planner
			std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
			this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			bool planForDynamicObstacle = false;
			if (obstaclesPos.size() != 0){
				planForDynamicObstacle = true;
			}

			bool planForSideWall = false;
			if (this->sideWallPoints_.size() != 0){
				planForSideWall = true;
			}
			bool replan = (this->td_.needReplan(1.0/2.0)) or this->isWallDetected() or (not this->trajValid_) or planForDynamicObstacle or planForSideWall;
			if (replan){
				nav_msgs::Path simplePath;
				geometry_msgs::PoseStamped pStart, pGoal;
				pStart.pose = this->odom_.pose.pose;

				// get the moving direction
				// this->moveToOrientation(this->getMovingDirection());
			

				pGoal = this->getForwardGoal();
				std::vector<geometry_msgs::PoseStamped> pathVec {pStart, pGoal};
				simplePath.poses = pathVec;
				this->pwlTraj_->updatePath(simplePath);
				this->pwlTraj_->makePlan(this->pwlTrajMsg_, 0.1);	

				std::vector<Eigen::Vector3d> startEndCondition;		
				this->getStartEndConditions(startEndCondition);
				bool updateSuccess = false;
				updateSuccess = this->bsplineTraj_->updatePath(this->pwlTrajMsg_, startEndCondition);

				if (updateSuccess){
					nav_msgs::Path bsplineTrajMsgTemp;
					bool planSuccess = this->bsplineTraj_->makePlan(bsplineTrajMsgTemp);
					if (planSuccess){
						this->bsplineTrajMsg_ = bsplineTrajMsgTemp;
						this->td_.updateTrajectory(this->bsplineTrajMsg_, this->bsplineTraj_->getDuration());
						this->countBsplineFailure_ = 0; // reset failure count
						this->trajValid_ = true;
					}
					else{
						this->td_.stop(this->odom_.pose.pose);
						++this->countBsplineFailure_;
					}
				}
			}
			this->prevState_ = this->flightState_;

			// if reach the wall, change the state to INSPECT
			if (this->isReach(this->getForwardGoal(), false)){
				if (this->isWallDetected() or this->inspectionGoalGiven_){
					this->td_.stop(this->odom_.pose.pose);
					this->useYaw_ = false;
					this->changeState(FLIGHT_STATE::INSPECT);
					cout << "[AutoFlight]: Switch from forward to inspection." << endl;
					cout << "[AutoFlight]: Start Inspection..." << endl;
					return;
				}
			}

			// if multiple times cannot navigate to the goal, change the state to EXPLORE
			if (this->countBsplineFailure_ > 3){
				cout << "[AutoFlight]: Wait for some time..." << endl;
				// wait for 2 seconds
				ros::Rate r (10);
				ros::Time startTime = ros::Time::now();
				ros::Time endTime;
				while (ros::ok()){
					endTime = ros::Time::now();
					if ((endTime - startTime).toSec() > 0.5){
						break;
					}
					r.sleep();
				}
			}

			if (this->countBsplineFailure_ > 3 and not this->isWallDetected()){
				this->useYaw_ = false;
				this->changeState(FLIGHT_STATE::EXPLORE); // for development
				cout << "[AutoFlight]: Switch from forward to explore." << endl;
				return;
			}



		}

		if (this->flightState_ == FLIGHT_STATE::EXPLORE){
			if (this->prevState_ == FLIGHT_STATE::FORWARD){
			// generate new local exploration goal
				Eigen::Vector3d pGoalExplore;
				bool bestViewPointSuccess = this->getBestViewPoint(pGoalExplore);
				if (bestViewPointSuccess){
					geometry_msgs::PoseStamped psGoalExplore = this->eigen2ps(pGoalExplore);
					this->rrtPlanner_->updateStart(this->odom_.pose.pose);
					this->rrtPlanner_->updateGoal(psGoalExplore.pose);
					this->rrtPlanner_->makePlan(this->rrtPathMsg_);	
					this->prevState_ = this->flightState_;
				}
				else{
					cout << "[AutoFlight]: Looking around to increase map range..." << endl;
					double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
					this->moveToOrientation(currYaw + PI_const/2.0);
					this->moveToOrientation(currYaw - PI_const/2.0);
					this->moveToOrientation(currYaw);
				}
				return;
			}

			if (this->prevState_ == FLIGHT_STATE::EXPLORE){
				std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
				this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
				bool planForDynamicObstacle = false;
				if (obstaclesPos.size() != 0){
					planForDynamicObstacle = true;
				}

				bool replan = (this->td_.needReplan(1.0/2.0)) or (not this->trajValid_) or planForDynamicObstacle;
				if (replan){
					// get the latest global waypoint path
					nav_msgs::Path latestGLobalPath = this->getLatestGlobalPath();
					double globalPathLength = this->getPathLength(latestGLobalPath);
					double minLength = 3.0;

					bool polySuccess = false;
					bool updateSuccess = false; // update success for bspline
					if (globalPathLength >= minLength){// if the lastest global path is long enough, we apply the polynomial trajectory
						this->polyTraj_->updatePath(latestGLobalPath);
						geometry_msgs::Twist vel;
						double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
						Eigen::Vector3d startCondition (cos(currYaw), sin(currYaw), 0);
						startCondition *= this->desiredVel_;
						vel.linear.x = startCondition(0);  vel.linear.y = startCondition(1);  vel.linear.z = startCondition(2); 
						this->polyTraj_->updateInitVel(vel);
						polySuccess = this->polyTraj_->makePlan(this->polyTrajMsg_);

						if (polySuccess){
							std::vector<Eigen::Vector3d> startEndCondition;		
							this->getStartEndConditions(startEndCondition);
							updateSuccess = this->bsplineTraj_->updatePath(this->polyTrajMsg_, startEndCondition);
						}
					}

					if (globalPathLength < minLength or polySuccess == false){// if it is very short, we use piecewise linear trajectory 
						nav_msgs::Path simplePath;
						geometry_msgs::PoseStamped pStart, pGoal;
						pStart.pose = this->odom_.pose.pose;
						pGoal = latestGLobalPath.poses.back();
			
						std::vector<geometry_msgs::PoseStamped> pathVec {pStart, pGoal};
						simplePath.poses = pathVec;
						this->pwlTraj_->updatePath(simplePath);
						this->pwlTraj_->makePlan(this->pwlTrajMsg_, 0.1);	

						std::vector<Eigen::Vector3d> startEndCondition;		
						this->getStartEndConditions(startEndCondition);
						updateSuccess = this->bsplineTraj_->updatePath(this->pwlTrajMsg_, startEndCondition);
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
			// change to forward if finish current exploration
			if (this->isReach(this->td_.trajectory.poses.back(), false) and this->prevState_ == FLIGHT_STATE::EXPLORE){
				this->prevState_ = this->flightState_;
				this->changeState(FLIGHT_STATE::FORWARD);
				cout << "[AutoFlight]: Switch from explore to forward." << endl;
				return;
			}
			this->prevState_ = this->flightState_;
		}

		if (this->flightState_ == FLIGHT_STATE::INSPECT){
			// generate zig-zag path and exexute. 

			if (not this->inspectionGoalGiven_){
				if (not this->inspectionWidthGiven_){
					// 1. check surroundings at each height
					this->checkSurroundings();
					// 2. start inspection
					this->inspectZigZag();
				}
				else{
					this->moveToPosition(Eigen::Vector3d (this->odom_.pose.pose.position.x, 0, this->takeoffHgt_), this->inspectionVel_);
					this->inspectZigZagRange();
				}

				
			}
			else{
				this->moveToOrientation(this->inspectionOrientation_);
				this->inspectZigZagRange();
			}

			// 3. directly change to back
			this->prevState_ = this->flightState_;
			this->changeState(FLIGHT_STATE::BACKWARD);
			cout << "[AutoFlight]: Switch from inspection to backward." << endl;
			
			return;
		}

		if (this->flightState_ == FLIGHT_STATE::BACKWARD){
			if (this->prevState_ == FLIGHT_STATE::INSPECT){
				double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);

				// turn back
				if (not this->inspectionGoalGiven_){
					this->moveToOrientation(currYaw-PI_const);
				}
				else{
					this->moveToOrientation(-PI_const);
				}

				// generate global waypoint path. set new goal to the origin position
				geometry_msgs::PoseStamped psBack = this->eigen2ps(Eigen::Vector3d (1.5, 0, this->takeoffHgt_));
				this->rrtPlanner_->updateStart(this->odom_.pose.pose);
				this->rrtPlanner_->updateGoal(psBack.pose);
				this->rrtPlanner_->makePlan(this->rrtPathMsg_);
			}

			if (this->prevState_ == FLIGHT_STATE::BACKWARD){
				this->useYaw_ = true;
				std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
				this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
				bool planForDynamicObstacle = false;
				if (obstaclesPos.size() != 0){
					planForDynamicObstacle = true;
				}
				bool replan = (this->td_.needReplan(1.0/2.0)) or (not this->trajValid_) or planForDynamicObstacle;
				if (replan){
					// get the latest global waypoint path
					nav_msgs::Path latestGLobalPath = this->getLatestGlobalPath();
					double globalPathLength = this->getPathLength(latestGLobalPath);
					double minLength = 3.0;

					bool polySuccess = false;
					bool updateSuccess = false; // update success for bspline
					if (globalPathLength >= minLength){// if the lastest global path is long enough, we apply the polynomial trajectory
						this->polyTraj_->updatePath(latestGLobalPath);
						geometry_msgs::Twist vel;
						double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
						Eigen::Vector3d startCondition (cos(currYaw), sin(currYaw), 0);
						startCondition *= this->desiredVel_;
						vel.linear.x = startCondition(0);  vel.linear.y = startCondition(1);  vel.linear.z = startCondition(2); 
						this->polyTraj_->updateInitVel(vel);
						polySuccess = this->polyTraj_->makePlan(this->polyTrajMsg_);

						if (polySuccess){
							std::vector<Eigen::Vector3d> startEndCondition;		
							this->getStartEndConditions(startEndCondition);
							updateSuccess = this->bsplineTraj_->updatePath(this->polyTrajMsg_, startEndCondition);
						}
					}

					if (globalPathLength < minLength or polySuccess == false){// if it is very short, we use piecewise linear trajectory 
						nav_msgs::Path simplePath;
						geometry_msgs::PoseStamped pStart, pGoal;
						pStart.pose = this->odom_.pose.pose;
						pGoal = latestGLobalPath.poses.back();
			
						std::vector<geometry_msgs::PoseStamped> pathVec {pStart, pGoal};
						simplePath.poses = pathVec;
						this->pwlTraj_->updatePath(simplePath);
						this->pwlTraj_->makePlan(this->pwlTrajMsg_, 0.1);	

						std::vector<Eigen::Vector3d> startEndCondition;		
						this->getStartEndConditions(startEndCondition);
						updateSuccess = this->bsplineTraj_->updatePath(this->pwlTrajMsg_, startEndCondition);
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
			this->prevState_ = this->flightState_;
			return;
		}

	}

	void dynamicInspection::trajExeCB(const ros::TimerEvent&){
		if (not this->td_.init){
			return;
		}
		if (not this->useYaw_){
			this->updateTarget(this->td_.getPoseWithoutYaw(this->odom_.pose.pose));
		}
		else{
			this->updateTarget(this->td_.getPose(this->odom_.pose.pose));	
		}
	}

	void dynamicInspection::checkWallCB(const ros::TimerEvent&){
		// use current robot position, check the occcupancy of the predefined wall size bounding box

		// const double maxWallWidth = 10.0; // The maximum width of the wall
		// const double maxWallHeight = 10.0; // the maximum height of the wall
		const double maxThickness = 3.0; // The maximum thickness of the wall

		// 1. find the occupied point in front of the robot
		Eigen::Vector3d currPos (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		// Eigen::Vector3d direction (1, 0, 0); // forward raycast
		double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		Eigen::Vector3d direction (cos(currYaw), sin(currYaw), 0);
		Eigen::Vector3d leftDirection (cos(currYaw+PI_const/2.0), sin(currYaw+PI_const/2.0), 0);
		Eigen::Vector3d rightDirection (cos(currYaw-PI_const/2.0), sin(currYaw-PI_const/2.0), 0);


		Eigen::Vector3d endPos;
		double maxRayLength = 5.0;
		bool castRaySuccess = this->map_->castRay(currPos, direction, endPos, maxRayLength);
		
		if (castRaySuccess){
			// search for the wall region
			double leftDistMax = 0.0;
			double rightDistMax = 0.0;
			double zpDistMax = 0.0;
			double znDistMax = currPos(2); // assume the wall start from 0 

			double searchThickness = maxThickness;
			const double searchResolution = 0.3;
			for (double t=0.0; t<maxThickness; t+=searchResolution){
				Eigen::Vector3d p = endPos + direction * t;
				Eigen::Vector3d pLeft, pRight, pZP, pZN;
				const double maxRayLengthWall = 7.0;
				
				// cast ray in left (or Y plus) direction
				bool leftSuccess = this->castRayOccupied(p, leftDirection, pLeft, maxRayLengthWall);
				double leftDist = (pLeft - p).norm();

				// cast ray in right (or Y minus) direction
				bool rightSuccess = this->castRayOccupied(p, rightDirection, pRight, maxRayLengthWall);
				double rightDist = (pRight - p).norm();

				// cast ray in +z direction
				bool zpSuccess = this->castRayOccupied(p, Eigen::Vector3d (0, 0, 1), pZP, maxRayLengthWall);
				double zpDist = (pZP - p).norm();


				if (leftDist > leftDistMax){
					leftDistMax = leftDist; // update max distance for left cast
				}

				if (rightDist > rightDistMax){
					rightDistMax = rightDist;
				}

				if (zpDist > zpDistMax){
					zpDistMax = zpDist;
				}


				bool noWall = (not leftSuccess) and (not rightSuccess) and (not zpSuccess);
				if (noWall){
					searchThickness = t;
					break;
				}
			}

			// find corresponding 3D points for the front wall
			Eigen::Vector3d pFLT = endPos + leftDirection * leftDistMax + Eigen::Vector3d (0, 0, 1) * zpDistMax;  // front left top
			Eigen::Vector3d pFLB = endPos + leftDirection * leftDistMax + Eigen::Vector3d (0, 0, -1) * znDistMax; // front left bottom
			Eigen::Vector3d pFRT = endPos + rightDirection * rightDistMax + Eigen::Vector3d (0, 0, 1) * zpDistMax; // front right top
			Eigen::Vector3d pFRB = endPos + rightDirection * rightDistMax + Eigen::Vector3d (0, 0, -1) * znDistMax; // front right bottom
			Eigen::Vector3d pRLT = endPos + leftDirection * leftDistMax + Eigen::Vector3d (0, 0, 1) * zpDistMax + searchThickness * direction; // rear left top
			Eigen::Vector3d pRLB = endPos + leftDirection * leftDistMax + Eigen::Vector3d (0, 0, -1) * znDistMax + searchThickness * direction; // rear left bottom
			Eigen::Vector3d pRRT = endPos + rightDirection * rightDistMax + Eigen::Vector3d (0, 0, 1) * zpDistMax + searchThickness * direction; // rear right top
			Eigen::Vector3d pRRB = endPos + rightDirection * rightDistMax + Eigen::Vector3d (0, 0, -1) * znDistMax + searchThickness * direction; // real right bottom

			std::vector<Eigen::Vector3d> frontWallPoints {pFLT, pFLB, pFRT, pFRB, pRLT, pRLB, pRRT, pRRB};
			this->updateFrontWallPoints(frontWallPoints);
			this->updateFrontWallDim(rightDistMax+leftDistMax, searchThickness, zpDistMax+znDistMax);
		}
		else{
			Eigen::Vector3d dummyPoint (0, 0, 0);
			std::vector<Eigen::Vector3d> frontWallPoints {dummyPoint, dummyPoint, dummyPoint, dummyPoint, dummyPoint, dummyPoint, dummyPoint, dummyPoint};
			this->updateFrontWallPoints(frontWallPoints);
			this->updateFrontWallDim(0, 0, 0);
		}
	}

	void dynamicInspection::sideWallRaycastCB(const ros::TimerEvent&){
		// timing
		// ros::Time startTime = ros::Time::now();

		// perform 360 degree ray casting
		int rayNum = 16 * 2;
		std::vector<Eigen::Vector3d> rayEndVec;
		std::vector<int> rayIDVec;
		this->sideCast(rayNum, rayEndVec, rayIDVec);


		// a. get the potential wall points
		std::vector<std::vector<Eigen::Vector3d>> potentialWallVec;
		this->getPotentialWallPoints(rayNum, rayEndVec, rayIDVec, potentialWallVec);


		// b. for each potential wall, check the length of the wall and filter out the invalid potential wall
		this->checkPotentialWallLength(potentialWallVec);

		// c. check the angle 
		this->checkPotentialWallAngle(potentialWallVec);

		// d. get the final wall
		std::vector<Eigen::Vector3d> finalSideWallPoints;
		this->getSideWallPoints(potentialWallVec, finalSideWallPoints);

		// ros::Time endTime = ros::Time::now();
		// double duration = (endTime - startTime).toSec();
		// cout << "[AutoFlight]: " << "Side wall raycast time: " << duration << endl;

	}

	void dynamicInspection::collisionCheckCB(const ros::TimerEvent&){
		if (this->td_.currTrajectory.poses.size() == 0) return;
		nav_msgs::Path currTrajectory = this->td_.currTrajectory;

		// we only check for static obstacle collision
		for (geometry_msgs::PoseStamped ps : currTrajectory.poses){
			Eigen::Vector3d p (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
			if (this->map_->isInflatedOccupied(p)){
				this->trajValid_ = false;
				return;
			}
		}
		this->trajValid_ = true;
	}

	void dynamicInspection::visCB(const ros::TimerEvent&){
		this->goalPub_.publish(this->goal_);
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

		if (this->frontWallPoints_.size() != 0){
			this->getWallVisMsg(this->wallVisMsg_);
			this->wallVisPub_.publish(this->wallVisMsg_);
		}

		// visualzize raycasting image
		this->getSideWallRaycastMsg(this->sideWallRaycastImg_);
		this->sideWallRaycastVisPub_.publish(this->sideWallRaycastImg_);
	}


	geometry_msgs::PoseStamped dynamicInspection::getForwardGoal(){
		// if user has specified the inspection goal location, follow the goal
		if (this->inspectionGoalGiven_){
			geometry_msgs::PoseStamped goal;
			goal.pose.position.x = this->inspectionGoal_(0);
			goal.pose.position.y = this->inspectionGoal_(1);
			goal.pose.position.z = this->inspectionGoal_(2);
			this->goal_ = goal;
			return goal;
		}


		// if not meet wall, the goal will always be aggresive (+10 meter in front of the robot)
		bool wallDetected = this->isWallDetected();
		geometry_msgs::PoseStamped goal;
		Eigen::Vector3d pGoal;
		Eigen::Vector3d pCurr (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);


		double targetYaw = this->getMovingDirection();
		Eigen::Vector3d direction (cos(targetYaw), sin(targetYaw), 0);

		// double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		// Eigen::Vector3d direction (cos(currYaw), sin(currYaw), 0);
		
		if (wallDetected){
			double maxRayLength = 7.0;
			Eigen::Vector3d pStart (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
			Eigen::Vector3d pEnd;
			this->map_->castRay(pStart, direction, pEnd, maxRayLength);
			double dist = (pEnd - pCurr).norm();
			if (dist <= this->safeDistance_){
				pGoal = pCurr;
			}
			else{
				pGoal = pEnd - direction * this->safeDistance_;
			}
		}
		else{
			pGoal = pCurr + direction * 10.0;
		}

		goal.pose.position.x = pGoal(0);
		goal.pose.position.y = pGoal(1);
		goal.pose.position.z = pGoal(2);
		goal.pose.orientation = AutoFlight::quaternion_from_rpy(0, 0, targetYaw);
		// goal.pose.orientation = AutoFlight::quaternion_from_rpy(0, 0, currYaw);

		this->goal_ = goal;
		return goal;
	}

	nav_msgs::Path dynamicInspection::getLatestGlobalPath(){
		nav_msgs::Path latestPath;

		int nextIdx = this->rrtPathMsg_.poses.size()-1;
		int idx = 0;
		Eigen::Vector3d pCurr (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		Eigen::Vector3d directionCurr (cos(currYaw), sin(currYaw), 0);
		double minDist = 100000;
		for (geometry_msgs::PoseStamped p : this->rrtPathMsg_.poses){
			Eigen::Vector3d pEig (p.pose.position.x, p.pose.position.y, p.pose.position.z);
			Eigen::Vector3d pDiff = pEig - pCurr;

			double dist = (pEig - pCurr).norm();
			if (dist < minDist and dist > 1.0 and trajPlanner::angleBetweenVectors(directionCurr, pDiff) < PI_const/4){
				nextIdx = idx;
				minDist = dist;
			}
			++idx;
		}

		geometry_msgs::PoseStamped psCurr;
		psCurr.pose = this->odom_.pose.pose;
		latestPath.poses.push_back(psCurr);
		for (size_t i=nextIdx; i<this->rrtPathMsg_.poses.size(); ++i){
			latestPath.poses.push_back(this->rrtPathMsg_.poses[i]);
		}
		return latestPath;
	}

	void dynamicInspection::getStartEndConditions(std::vector<Eigen::Vector3d>& startEndCondition){
		double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		Eigen::Vector3d startCondition (cos(currYaw), sin(currYaw), 0);
		startCondition *= this->desiredVel_;
		startEndCondition.push_back(startCondition); // start vel
		startEndCondition.push_back(Eigen::Vector3d (0, 0, 0)); //start acc condition
		startEndCondition.push_back(Eigen::Vector3d (0, 0, 0)); //end vel condition
		startEndCondition.push_back(Eigen::Vector3d (0, 0, 0)); //end acc condition
	}


	void dynamicInspection::changeState(const FLIGHT_STATE& flightState){
		this->flightState_ = flightState;
	}

	bool dynamicInspection::moveToPosition(const geometry_msgs::Point& position){
		geometry_msgs::PoseStamped psStart, psGoal;
		psGoal.pose.position = position;
		psGoal.pose.orientation = this->odom_.pose.pose.orientation;
		psStart.pose = this->odom_.pose.pose;

		std::vector<geometry_msgs::PoseStamped> linePathVec;
		linePathVec.push_back(psStart);
		linePathVec.push_back(psGoal);
		nav_msgs::Path linePath;
		linePath.poses = linePathVec;

		this->pwlTraj_->updatePath(linePath);
		this->pwlTraj_->makePlan(this->pwlTrajMsg_, 0.1);	
		this->td_.updateTrajectory(this->pwlTrajMsg_, this->pwlTraj_->getDuration());

		ros::Rate r (50);
		while (ros::ok() and not this->isReach(psGoal, false)){
			ros::spinOnce();
			r.sleep();
		}
		return true;
	}

	bool dynamicInspection::moveToPosition(const geometry_msgs::Point& position, double vel){
		geometry_msgs::PoseStamped psStart, psGoal;
		psGoal.pose.position = position;
		psGoal.pose.orientation = this->odom_.pose.pose.orientation;
		psStart.pose = this->odom_.pose.pose;

		std::vector<geometry_msgs::PoseStamped> linePathVec;
		linePathVec.push_back(psStart);
		linePathVec.push_back(psGoal);
		nav_msgs::Path linePath;
		linePath.poses = linePathVec;

		this->pwlTraj_->updatePath(linePath, vel);
		this->pwlTraj_->makePlan(this->pwlTrajMsg_, 0.1);	
		this->td_.updateTrajectory(this->pwlTrajMsg_, this->pwlTraj_->getDuration());

		ros::Rate r (50);
		while (ros::ok() and not this->isReach(psGoal, false)){
			ros::spinOnce();
			r.sleep();
		}
		return true;
	}

	bool dynamicInspection::moveToPosition(const Eigen::Vector3d& position){
		geometry_msgs::Point p;
		p.x = position(0);
		p.y = position(1);
		p.z = position(2);
		this->moveToPosition(p);
		return true;
	}

	bool dynamicInspection::moveToPosition(const Eigen::Vector3d& position, double vel){
		geometry_msgs::Point p;
		p.x = position(0);
		p.y = position(1);
		p.z = position(2);
		this->moveToPosition(p, vel);
		return true;
	}
	
	bool dynamicInspection::moveToOrientation(const geometry_msgs::Quaternion& orientation){
		double yawTgt = AutoFlight::rpy_from_quaternion(orientation);
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


		double t = 0.0;
		double startTime = 0.0; double endTime = yawDiffAbs/this->desiredAngularVel_;

		nav_msgs::Path rotationPath;
		std::vector<geometry_msgs::PoseStamped> rotationPathVec;
		while (t <= endTime){
			double currYawTgt = yawCurr + (double) direction * (t-startTime)/(endTime-startTime) * yawDiffAbs;
			geometry_msgs::Quaternion quatT = trajPlanner::quaternion_from_rpy(0, 0, currYawTgt);
			geometry_msgs::PoseStamped psT = ps;
			psT.pose.orientation = quatT;
			rotationPathVec.push_back(psT);
			t += 0.1;
		}
		rotationPath.poses = rotationPathVec;
		this->useYaw_ = true;
		this->td_.updateTrajectory(rotationPath, endTime);

		ros::Rate r (50);
		while (ros::ok() and not this->isReach(ps)){
			ros::spinOnce();
			r.sleep();
		}
		this->useYaw_ = false;
		return true;
	}

	bool dynamicInspection::moveToOrientation(double yaw){
		geometry_msgs::Quaternion quat = AutoFlight::quaternion_from_rpy(0, 0, yaw);
		this->moveToOrientation(quat);
		return true;
	}

	double dynamicInspection::makePWLTraj(const std::vector<geometry_msgs::PoseStamped>& waypoints, nav_msgs::Path& resultPath){
		nav_msgs::Path waypointsMsg;
		waypointsMsg.poses = waypoints;
		this->pwlTraj_->updatePath(waypointsMsg, true);
		this->pwlTraj_->makePlan(resultPath, 0.1);
		return this->pwlTraj_->getDuration();
	}

	double dynamicInspection::getPathLength(const nav_msgs::Path& path){
		double length = 0.0;
		for (size_t i=0; i<path.poses.size()-1; ++i){
			geometry_msgs::PoseStamped ps1 = path.poses[i];
			geometry_msgs::PoseStamped ps2 = path.poses[i+1];
			Eigen::Vector3d p1 (ps1.pose.position.x, ps1.pose.position.y, ps1.pose.position.z);
			Eigen::Vector3d p2 (ps2.pose.position.x, ps2.pose.position.y, ps2.pose.position.z);
			length += (p1 - p2).norm();
		}
		return length;
	}


	double dynamicInspection::makePWLTraj(const std::vector<geometry_msgs::PoseStamped>& waypoints, double desiredVel, nav_msgs::Path& resultPath){
		nav_msgs::Path waypointsMsg;
		waypointsMsg.poses = waypoints;
		this->pwlTraj_->updatePath(waypointsMsg, desiredVel, true);
		this->pwlTraj_->makePlan(resultPath, 0.1);
		return this->pwlTraj_->getDuration();
	}

	bool dynamicInspection::getBestViewPoint(Eigen::Vector3d& bestPoint){
		int bestUnknown = 0;
		for (int i=0; i<this->exploreSampleNum_; ++i){
			Eigen::Vector3d pRandom;
			bool sampleSuccess = this->randomSample(pRandom);
			if (not sampleSuccess){
				return false;
			}
			int countUnknown = this->countUnknownFOV(pRandom, 0);
			if (countUnknown > bestUnknown){
				bestUnknown = countUnknown;
				bestPoint = pRandom;
			}
		}
		return true;
	}

	bool  dynamicInspection::randomSample(Eigen::Vector3d& pSample){
		Eigen::Vector3d mapSizeMin, mapSizeMax;
		this->map_->getMapRange(mapSizeMin, mapSizeMax);

		ros::Time startTime = ros::Time::now();
		ros::Time endTime;
		while (ros::ok()){
			endTime = ros::Time::now();
			if ((endTime - startTime).toSec() > 1.0){
				cout << "[AutoFlight]: Explore random sample timeout." << endl;
				return false;
			}
			pSample(0) = AutoFlight::randomNumber(this->odom_.pose.pose.position.x, mapSizeMax(0));
			pSample(1) = AutoFlight::randomNumber(mapSizeMin(1), mapSizeMax(1));
			pSample(2) = this->takeoffHgt_;
			
			if (not this->map_->isInflatedOccupied(pSample) and not this->map_->isUnknown(pSample) and this->satisfyWallDistance(pSample)){
				return true;			
			}
		}
		return false;
	}

	bool dynamicInspection::satisfyWallDistance(const Eigen::Vector3d& p){
		// check front and side
		Eigen::Vector3d pEndFront, pEndLeft, pEndRight;
		bool ignoreUnknown = false; 
		bool castSuccessFront = this->map_->castRay(p, Eigen::Vector3d (1, 0, 0), pEndFront, this->safeDistance_, ignoreUnknown);
		// bool castSuccessLeft = this->map_->castRay(p, Eigen::Vector3d (0, 1, 0), pEndLeft, this->sideSafeDistance_, ignoreUnknown);
		// bool castSuccessRight = this->map_->castRay(p, Eigen::Vector3d (0, -1, 0), pEndRight, this->sideSafeDistance_, ignoreUnknown);
		bool castSuccessLeft = false;
		bool castSuccessRight = false;


		if (castSuccessFront or castSuccessRight or castSuccessLeft){
			return false;
		}
		else{
			return true;
		}
	}

	int dynamicInspection::countUnknownFOV(const Eigen::Vector3d& p, double yaw){
		double height = 2 * tan(this->sensorAngleV_/2.0) * this->sensorRange_;
		double width = 2 * tan(this->sensorAngleH_/2.0) * this->sensorRange_;

		// create bounding box for search
		double xmin = p(0);              double xmax = p(0) + this->sensorRange_;
		double ymin = p(1) - width/2.0;  double ymax = p(1) + width/2.0;
		double zmin = p(2) - height/2.0; double zmax = p(2) + height/2.0;
		
		Eigen::Vector3d faceDirection (cos(yaw), sin(yaw), 0);
		int countUnknown = 0;
		for (double x=xmin; x<=xmax; x+=this->map_->getRes()){
			for (double y=ymin; y<=ymax; y+=this->map_->getRes()){
				for (double z=zmin; z<=zmax; z+=this->map_->getRes()){
					Eigen::Vector3d pQuery(x, y, z);
					if (not this->map_->isInMap(pQuery)){
						continue;
					}
					else{
						if (this->map_->isUnknown(pQuery)){ // need to be unknown voxel
							Eigen::Vector3d direction = pQuery - p;
							Eigen::Vector3d directionH = direction; directionH(2) = 0;
							Eigen::Vector3d directionV = direction; directionV(1) = 0;

							double angleH = trajPlanner::angleBetweenVectors(directionH, faceDirection);
							double angleV = trajPlanner::angleBetweenVectors(directionV, faceDirection);

							if (angleH <= this->sensorAngleH_/2.0 and angleV <= this->sensorAngleV_/2.0){
								++countUnknown;
							}
						}
					}
				}
			}
		}
		return countUnknown;
	}

	void dynamicInspection::setStartPositionFree(){
		Eigen::Vector3d lowerPos, upperPos;
		Eigen::Vector3d centerPos (0 , 0, this->takeoffHgt_);
		lowerPos = centerPos - Eigen::Vector3d (0.5, 0.5, 0.5);
		upperPos = centerPos + Eigen::Vector3d (0.5, 0.5, 0.5);
		this->map_->freeRegion(lowerPos, upperPos);
	}

	bool dynamicInspection::castRayOccupied(const Eigen::Vector3d& start, const Eigen::Vector3d& direction, Eigen::Vector3d& end, double maxRayLength){
		// return true if raycasting successfully find the endpoint, otherwise return false
		if (not this->map_->isInflatedOccupied(start)){ // 
			end = start;
			return false;
		}

		Eigen::Vector3d directionNormalized = direction/direction.norm(); // normalize the direction vector
		int num = ceil(maxRayLength/this->map_->getRes());
		for (int i=1; i<num; ++i){
			Eigen::Vector3d point = this->map_->getRes() * directionNormalized * i + start;
			if (not this->map_->isInflatedOccupied(point)){
				end = point;
				return true;
			}
		}
		end = start;
		return false;
	}

	bool dynamicInspection::isWallDetected(){
		if (this->frontWallPoints_.size() == 0){
			return false;
		}

		double area = this->frontWallDim_(0) * this->frontWallDim_(2);
		if (area > this->minWallArea_){
			return true;
		}
		else{
			return false;
		}
	}

	double dynamicInspection::getWallDistance(){
		Eigen::Vector3d frontWallCenter = (this->frontWallPoints_[0] + this->frontWallPoints_[1] + this->frontWallPoints_[2] + this->frontWallPoints_[3]) * 0.25;
		Eigen::Vector3d currPos (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		return (frontWallCenter - currPos).norm();
	}

	void dynamicInspection::updateWallRange(const std::vector<double>& wallRange){
		this->wallRange_ = wallRange;
	}

	void dynamicInspection::updateFrontWallPoints(const std::vector<Eigen::Vector3d>& frontWallPoints){
		this->frontWallPoints_ = frontWallPoints;
	}

	void dynamicInspection::updateFrontWallDim(double length, double width, double height){
		Eigen::Vector3d dim (length, width, height);
		this->frontWallDim_ = dim;
	}

	visualization_msgs::Marker dynamicInspection::getLineMarker(double x1, double y1, double z1, 
										  						double x2, double y2, double z2,
										  						int id, bool isWall){
		std::vector<geometry_msgs::Point> lineVec;
		geometry_msgs::Point p1, p2;
		p1.x = x1; p1.y = y1; p1.z = z1;
		p2.x = x2; p2.y = y2; p2.z = z2;
		lineVec.push_back(p1); 
		lineVec.push_back(p2);

		visualization_msgs::Marker lineMarker;
		lineMarker.header.frame_id = "map";
		lineMarker.header.stamp = ros::Time::now();
		lineMarker.ns = "inspection_target";
		lineMarker.id = id;
		lineMarker.type = visualization_msgs::Marker::LINE_LIST;
		lineMarker.action = visualization_msgs::Marker::ADD;
		lineMarker.points = lineVec;
		lineMarker.lifetime = ros::Duration(2);
		lineMarker.scale.x = 0.1;
		lineMarker.scale.y = 0.1;
		lineMarker.scale.z = 0.1;
		lineMarker.color.a = 1.0; // Don't forget to set the alpha!
		if (isWall){
			lineMarker.color.r = 1.0;
			lineMarker.color.g = 0.0;
			lineMarker.color.b = 0.0;
		}
		else{
			lineMarker.color.r = 1.0;
			lineMarker.color.g = 0.0;
			lineMarker.color.b = 1.0;
		}
		
		return lineMarker;
	}

	visualization_msgs::Marker dynamicInspection::getLineMarker(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, int id, bool isWall){
		double x1 = p1(0); 
		double y1 = p1(1);
		double z1 = p1(2);

		double x2 = p2(0); 
		double y2 = p2(1);
		double z2 = p2(2);
		return this->getLineMarker(x1, y1, z1, x2, y2, z2, id, isWall);
	}

	void dynamicInspection::getWallVisMsg(visualization_msgs::MarkerArray& msg){
		msg.markers.clear();


		bool isWall = this->isWallDetected();
		std::vector<visualization_msgs::Marker> visVec;
		visualization_msgs::Marker l1 = this->getLineMarker(this->frontWallPoints_[0], this->frontWallPoints_[1], 1, isWall);
		visualization_msgs::Marker l2 = this->getLineMarker(this->frontWallPoints_[0], this->frontWallPoints_[2], 2, isWall);
		visualization_msgs::Marker l3 = this->getLineMarker(this->frontWallPoints_[1], this->frontWallPoints_[3], 3, isWall);
		visualization_msgs::Marker l4 = this->getLineMarker(this->frontWallPoints_[2], this->frontWallPoints_[3], 4, isWall);
		visualization_msgs::Marker l5 = this->getLineMarker(this->frontWallPoints_[0], this->frontWallPoints_[4], 5, isWall);
		visualization_msgs::Marker l6 = this->getLineMarker(this->frontWallPoints_[1], this->frontWallPoints_[5], 6, isWall);
		visualization_msgs::Marker l7 = this->getLineMarker(this->frontWallPoints_[2], this->frontWallPoints_[6], 7, isWall);
		visualization_msgs::Marker l8 = this->getLineMarker(this->frontWallPoints_[3], this->frontWallPoints_[7], 8, isWall);
		visualization_msgs::Marker l9 = this->getLineMarker(this->frontWallPoints_[4], this->frontWallPoints_[5], 9, isWall);
		visualization_msgs::Marker l10 = this->getLineMarker(this->frontWallPoints_[4], this->frontWallPoints_[6], 10, isWall);
		visualization_msgs::Marker l11 = this->getLineMarker(this->frontWallPoints_[5], this->frontWallPoints_[7], 11, isWall);
		visualization_msgs::Marker l12 = this->getLineMarker(this->frontWallPoints_[6], this->frontWallPoints_[7], 12, isWall);

		// line 1-12
		visVec.push_back(l1);
		visVec.push_back(l2);
		visVec.push_back(l3);
		visVec.push_back(l4);
		visVec.push_back(l5);
		visVec.push_back(l6);
		visVec.push_back(l7);
		visVec.push_back(l8);
		visVec.push_back(l9);
		visVec.push_back(l10);
		visVec.push_back(l11);
		visVec.push_back(l12);

		msg.markers = visVec;
	}


	void dynamicInspection::checkSurroundings(){
		// 1. find all the height level
		std::vector<double> heightLevels;
		double currHeight = this->odom_.pose.pose.position.z;
		double heightTemp = currHeight;
		while (ros::ok() and heightTemp < this->inspectionHeight_){
			heightLevels.push_back(heightTemp);
			heightTemp += this->ascendStep_;
		}
		heightLevels.push_back(this->inspectionHeight_);

		// 2. for each height level check left and right
		double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		Eigen::Vector3d leftDirection (cos(currYaw + PI_const/2.0), sin(currYaw + PI_const/2.0), 0);
		Eigen::Vector3d rightDirection (cos(currYaw - PI_const/2.0), sin(currYaw - PI_const/2.0), 0);

		double maxRayLength = 7.0;
		ros::Rate r (50);

		for (size_t i=0; i<heightLevels.size(); ++i){
			Eigen::Vector3d pHeight (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, heightLevels[i]);

			// a. turn left
			Eigen::Vector3d leftEnd;
			bool castLeftSuccess = this->map_->castRay(pHeight, leftDirection, leftEnd, maxRayLength);
			
		
			if (not castLeftSuccess){
				this->moveToOrientation(currYaw + PI_const/2);
				// translation
				while (ros::ok() and not castLeftSuccess){
					geometry_msgs::PoseStamped pStart, pGoal;
					pStart.pose = this->odom_.pose.pose;
					Eigen::Vector3d pStartEig (pStart.pose.position.x, pStart.pose.position.y, pStart.pose.position.z);
					Eigen::Vector3d pGoalEig = pStartEig + 1.0 * leftDirection;
					pGoal = this->eigen2ps(pGoalEig); pGoal.pose.orientation = pStart.pose.orientation;
					std::vector<geometry_msgs::PoseStamped> pathVec {pStart, pGoal};
					double duration = this->makePWLTraj(pathVec, this->pwlTrajMsg_);
					this->td_.updateTrajectory(this->pwlTrajMsg_, duration);


					Eigen::Vector3d pCurr(this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
					castLeftSuccess = this->map_->castRay(pCurr, leftDirection, leftEnd, maxRayLength);

					ros::spinOnce();
					r.sleep();
				}
				this->moveToOrientation(currYaw);
			}
			

			// b. turn right
			Eigen::Vector3d rightEnd;
			bool castRightSuccess = this->map_->castRay(pHeight, rightDirection, rightEnd, maxRayLength);
			if (not castRightSuccess){
				this->moveToOrientation(currYaw - PI_const/2);
				while (ros::ok() and not castRightSuccess){
					geometry_msgs::PoseStamped pStart, pGoal;
					pStart.pose = this->odom_.pose.pose;
					Eigen::Vector3d pStartEig (pStart.pose.position.x, pStart.pose.position.y, pStart.pose.position.z);
					Eigen::Vector3d pGoalEig = pStartEig + 1.0 * rightDirection;
					pGoal = this->eigen2ps(pGoalEig); pGoal.pose.orientation = pStart.pose.orientation;
					std::vector<geometry_msgs::PoseStamped> pathVec {pStart, pGoal};
					double duration = this->makePWLTraj(pathVec, this->pwlTrajMsg_);
					this->td_.updateTrajectory(this->pwlTrajMsg_, duration);
					
					Eigen::Vector3d pCurr(this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
					castRightSuccess = this->map_->castRay(pCurr, rightDirection, rightEnd, maxRayLength);
					ros::spinOnce();
					r.sleep();
				}

				this->moveToOrientation(currYaw);
			}

			// c. back to the center of the wall
			Eigen::Vector3d pCenter = (rightEnd + leftEnd)/2.0;
			this->moveToPosition(pCenter, this->inspectionVel_);
			
			// d. move to next height
			if (i != heightLevels.size()-1){
				double height  = heightLevels[i+1];
				Eigen::Vector3d pNextHeight (pCenter(0), pCenter(1), height);
				this->moveToPosition(pNextHeight, this->inspectionVel_);
			}

		}
	}

	void dynamicInspection::inspectZigZag(){
		std::vector<geometry_msgs::PoseStamped> zigzagPathVec;

		// 1. find all height levels
		std::vector<double> heightLevels;
		
		double currHeight = this->odom_.pose.pose.position.z;
		double heightTemp = currHeight;
		while (ros::ok() and heightTemp > this->takeoffHgt_){
			heightLevels.push_back(heightTemp);
			heightTemp -= this->descendStep_;
		}
		heightLevels.push_back(this->takeoffHgt_);


		// 2. cast rays to two sides
		double currX = this->odom_.pose.pose.position.x;
		double currY = this->odom_.pose.pose.position.y;
		double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		Eigen::Vector3d leftDirection (cos(currYaw + PI_const/2.0), sin(currYaw + PI_const/2.0), 0);
		Eigen::Vector3d rightDirection (cos(currYaw - PI_const/2.0), sin(currYaw - PI_const/2.0), 0);
		double maxRayLength = 7.0;
		
		Eigen::Vector3d pStart (currX, currY, currHeight);
		geometry_msgs::PoseStamped psStart = this->eigen2ps(pStart);
		psStart.pose.orientation = AutoFlight::quaternion_from_rpy(0, 0, currYaw);
		zigzagPathVec.push_back(psStart);

		int count = 0;
		bool ignoreUnknown = true;
		for (double height : heightLevels){
			Eigen::Vector3d pCurr (currX, currY, height);

			// cast to left
			Eigen::Vector3d pLeftEnd;
			this->map_->castRay(pCurr, leftDirection, pLeftEnd, maxRayLength, ignoreUnknown);
			double leftDist = (pLeftEnd - pCurr).norm();
			if (leftDist < this->sideSafeDistance_){
				pLeftEnd = pCurr;
			}
			else{
				pLeftEnd = pLeftEnd - leftDirection * this->sideSafeDistance_;
			}			geometry_msgs::PoseStamped psLeft = this->eigen2ps(pLeftEnd);
			psLeft.pose.orientation = AutoFlight::quaternion_from_rpy(0, 0, currYaw);

			// cast to right
			Eigen::Vector3d pRightEnd;
			this->map_->castRay(pCurr, rightDirection, pRightEnd, maxRayLength, ignoreUnknown);
			double rightDist = (pRightEnd - pCurr).norm();
			if (rightDist < this->sideSafeDistance_){
				pRightEnd = pCurr;
			}
			else{
				pRightEnd = pRightEnd - rightDirection * this->sideSafeDistance_;
			}

			geometry_msgs::PoseStamped psRight = this->eigen2ps(pRightEnd);
			psRight.pose.orientation = AutoFlight::quaternion_from_rpy(0, 0, currYaw);

			if (count % 2 == 0){
				if (count != 0){
					geometry_msgs::PoseStamped psPrev = zigzagPathVec.back();
					psPrev.pose.position.z =  height;
					zigzagPathVec.push_back(psPrev);
				}
				zigzagPathVec.push_back(psLeft);
				zigzagPathVec.push_back(psRight);
			}
			else{
				geometry_msgs::PoseStamped psPrev = zigzagPathVec.back();
				psPrev.pose.position.z =  height;
				zigzagPathVec.push_back(psPrev);
				zigzagPathVec.push_back(psRight);
				zigzagPathVec.push_back(psLeft);
			}
			++count;
		}

		Eigen::Vector3d pEnd (currX, currY, this->takeoffHgt_);
		geometry_msgs::PoseStamped psEnd = this->eigen2ps(pEnd);
		psEnd.pose.orientation = AutoFlight::quaternion_from_rpy(0, 0, currYaw);
		zigzagPathVec.push_back(psEnd);


		double duration = this->makePWLTraj(zigzagPathVec, this->inspectionVel_, this->pwlTrajMsg_);
		this->td_.updateTrajectory(this->pwlTrajMsg_, duration);


		ros::Rate r (10);
		while ((ros::ok() and not (this->isReach(psEnd, false))) or (this->td_.getRemainTime() > 0)){
			ros::spinOnce();
			r.sleep();
		}
	}

	void dynamicInspection::inspectZigZagRange(){
		// 1. move to the desired height
		geometry_msgs::PoseStamped psHeight;
		psHeight.pose = this->odom_.pose.pose;
		psHeight.pose.position.z = this->inspectionHeight_;
		this->moveToPosition(psHeight.pose.position, this->inspectionVel_);

		std::vector<geometry_msgs::PoseStamped> zigzagPathVec;

		// 2. find all height levels
		std::vector<double> heightLevels;
		
		double currHeight = this->odom_.pose.pose.position.z;
		double heightTemp = currHeight;
		while (ros::ok() and heightTemp > this->takeoffHgt_){
			heightLevels.push_back(heightTemp);
			heightTemp -= this->descendStep_;
		}
		heightLevels.push_back(this->takeoffHgt_);

		
		// 3. use the given range to construct the path
		double inspectionOrientation = 0;
		if (this->inspectionGoalGiven_){
			inspectionOrientation = this->inspectionOrientation_;
		}
		geometry_msgs::Quaternion quat = AutoFlight::quaternion_from_rpy(0, 0, inspectionOrientation);	
		double currX = this->odom_.pose.pose.position.x;
		double currY = this->odom_.pose.pose.position.y;
		Eigen::Vector3d pStart (currX, currY, currHeight);
		geometry_msgs::PoseStamped psStart = this->eigen2ps(pStart);
		psStart.pose.orientation = quat;
		zigzagPathVec.push_back(psStart);
	
		int count = 0;
		for (double height : heightLevels){
			Eigen::Vector3d pLeft (currX - this->inspectionWidth_/2.0 * sin(inspectionOrientation), currY + this->inspectionWidth_/2.0 * cos(inspectionOrientation), height);
			Eigen::Vector3d pRight (currX + this->inspectionWidth_/2.0 * sin(inspectionOrientation), currY - this->inspectionWidth_/2.0 * cos(inspectionOrientation), height);
			geometry_msgs::PoseStamped psLeft = this->eigen2ps(pLeft);
			psLeft.pose.orientation = quat;
			geometry_msgs::PoseStamped psRight = this->eigen2ps(pRight);
			psRight.pose.orientation = quat;

			if (count % 2 == 0){
				if (count != 0){
					geometry_msgs::PoseStamped psPrev = zigzagPathVec.back();
					psPrev.pose.position.z =  height;
					zigzagPathVec.push_back(psPrev);					
				}
				zigzagPathVec.push_back(psLeft);
				zigzagPathVec.push_back(psRight);
			}
			else{
				geometry_msgs::PoseStamped psPrev = zigzagPathVec.back();
				psPrev.pose.position.z =  height;
				zigzagPathVec.push_back(psPrev);
				zigzagPathVec.push_back(psRight);
				zigzagPathVec.push_back(psLeft);				
			}
			++count;
		}

		Eigen::Vector3d pEnd (currX, currY, this->takeoffHgt_);
		geometry_msgs::PoseStamped psEnd = this->eigen2ps(pEnd);
		psEnd.pose.orientation = quat;
		zigzagPathVec.push_back(psEnd);


		double duration = this->makePWLTraj(zigzagPathVec, this->inspectionVel_, this->pwlTrajMsg_);
		this->td_.updateTrajectory(this->pwlTrajMsg_, duration);


		ros::Rate r (50);
		while ((ros::ok() and not (this->isReach(psEnd, false))) or (this->td_.getRemainTime() > 0)){
			ros::spinOnce();
			r.sleep();
		}
	}

	double dynamicInspection::getMovingDirection(){
		// if we did not detect the wall, we use the current direction
		double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		double targetYaw = currYaw;

		if (this->movingDirection_ != -12345.0){ // we have previous moving direction
			targetYaw = this->movingDirection_;
		}

		if (this->finalSideWallPoints_.size() != 0){
			Eigen::Vector3d lastWallPoint = this->finalSideWallPoints_[this->finalSideWallPoints_.size() - 1];
			Eigen::Vector3d secondLastWallPoint = this->finalSideWallPoints_[this->finalSideWallPoints_.size() - 2];
			targetYaw = atan2(lastWallPoint(1) - secondLastWallPoint(1), lastWallPoint(0) - secondLastWallPoint(0));
		}

		double angleDiff = AutoFlight::getAngleDiff(currYaw, targetYaw);
		double maxTurningAngle = 75.0 * PI_const/180.0;
		if (this->isWallDetected() or angleDiff > maxTurningAngle){
			targetYaw = this->movingDirection_;
		}
		
		this->movingDirection_ = targetYaw;
		return targetYaw;
	}

	void dynamicInspection::sideCast(int rayNum, std::vector<Eigen::Vector3d>& rayEndVec, std::vector<int>& rayIDVec){
		this->sideWallPointsAll_.resize(rayNum);
		double interval = (double) PI_const * 2.0 / (double) rayNum;

		// current position
		Eigen::Vector3d pCurr (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);

		// get directions for ray casting and cast ray
		double maxRayLength = this->sideWallRaycastRange_;
		bool ignoreUnknown = true;
		double angle = 0.0;
		for (int i=0; i<rayNum; ++i){
			Eigen::Vector3d rayDirection (cos(angle), sin(angle), 0.0);
			Eigen::Vector3d rayEnd;
			bool raycastSuccess = this->map_->castRay(pCurr, rayDirection, rayEnd, maxRayLength, ignoreUnknown);
			if (raycastSuccess){
				this->sideWallPointsAll_[i] = rayEnd;
				rayEndVec.push_back(rayEnd);
				rayIDVec.push_back(i);
			}
			angle += interval;
		}

		this->sideWallPoints_ = rayEndVec;


		// substract the raycasting points with the current position
		std::vector< Eigen::Vector3d> rayEndVecC;
		for (size_t i=0; i<rayEndVec.size(); ++i){
			Eigen::Vector3d rayEndC = rayEndVec[i] - pCurr;
			rayEndVecC.push_back(rayEndC);
		}
		this->sideWallPointsC_ = rayEndVecC;		
	}


	// TO debug
	void dynamicInspection::getPotentialWallPoints(int rayNum, const std::vector<Eigen::Vector3d>& rayEndVec, const std::vector<int>& rayIDVec, std::vector<std::vector<Eigen::Vector3d>>& potentialWallVec){
		// check whether it is as wall
		// get the potential wall points
		// first get the consecutive ray IDs
		std::vector<std::vector<int>> consecutiveIDVec;
		int currID, prevID;
		std::vector<int> tempVec;
		for (size_t i=0; i<rayIDVec.size(); ++i){
			currID = rayIDVec[i];
			if (i != 0){
				// check whether current ID is incremented from the previous one
				if (currID - prevID == 1){ // they are neighbors
					if (tempVec.size() == 0){
						tempVec.push_back(prevID);
					}
					tempVec.push_back(currID);
					if (currID == rayNum - 1){ // last in the ray
						consecutiveIDVec.push_back(tempVec);
					}
				}
				else{ // isn's neighbor
					// if the previous tempVec is not empty, we need to save it
					if (tempVec.size() != 0){
						consecutiveIDVec.push_back(tempVec);
						tempVec.clear();
					}
				}

			}
			prevID = currID;
		}

		if (consecutiveIDVec.size() == 0){ // no consecutive points
			return;
		}


		// check whether the first and last ID is in the consecutive vec. If they are, we need to merge two vector
		if (consecutiveIDVec[0][0] == 0 and (consecutiveIDVec.back()).back() == rayNum - 1){
			std::vector<std::vector<int>> newConsecutiveIDVec;
			std::vector<int> firstIDVec = consecutiveIDVec[0];
			std::vector<int> lastIDVec = consecutiveIDVec.back();
			lastIDVec.insert(lastIDVec.end(), firstIDVec.begin(), firstIDVec.end());

			newConsecutiveIDVec.push_back(lastIDVec);
			for (size_t i=1; i<consecutiveIDVec.size()-1; ++i){
				newConsecutiveIDVec.push_back(consecutiveIDVec[i]);
			}
			consecutiveIDVec = newConsecutiveIDVec;
		}

		// get the potential wall points
		for (size_t i=0; i<consecutiveIDVec.size(); ++i){
			std::vector<Eigen::Vector3d> points;
			for (size_t j=0; j<consecutiveIDVec[i].size(); ++j){
				int ID = consecutiveIDVec[i][j];
				points.push_back(this->sideWallPointsAll_[ID]);
			}
			potentialWallVec.push_back(points);
		}
	}

	void dynamicInspection::checkPotentialWallLength(std::vector<std::vector<Eigen::Vector3d>>& potentialWallVec){
		std::vector<std::vector<Eigen::Vector3d>> prevPotentialWallVec = potentialWallVec;
		potentialWallVec.clear();
		for (size_t i=0; i<prevPotentialWallVec.size(); ++i){
			double wallLength = this->calculateWallLength(prevPotentialWallVec[i]);
			if (wallLength >= this->sideWallLengthThresh_){
				potentialWallVec.push_back(prevPotentialWallVec[i]);
			}
		}
	}

	double dynamicInspection::calculateWallLength(const std::vector<Eigen::Vector3d>& points){
		double totalLength = 0.0;
		for (size_t i=0; i<points.size()-1; ++i){
			Eigen::Vector3d cp = points[i];
			Eigen::Vector3d np = points[i+1];
			totalLength += (np - cp).norm();
		}
		return totalLength;
	}

	void dynamicInspection::checkPotentialWallAngle(std::vector<std::vector<Eigen::Vector3d>>& potentialWallVec){
		std::vector<std::vector<Eigen::Vector3d>> prevPotentialWallVec = potentialWallVec;
		potentialWallVec.clear();
		for (size_t i=0; i<prevPotentialWallVec.size(); ++i){
			double minAngle = this->checkWallPointMinAngle(prevPotentialWallVec[i]);
			if (minAngle >= this->sideWallAngleThresh_){
				potentialWallVec.push_back(prevPotentialWallVec[i]);
			}
		}
	}

	double dynamicInspection::checkWallPointMinAngle(const std::vector<Eigen::Vector3d>& points){
		if (points.size() <= 2){
			return 0.0;
		}
		double minAngle = 2 * PI_const;
		Eigen::Vector3d startP = points[0];
		Eigen::Vector3d endP = points.back();
		for (size_t i=1; i<points.size()-1; ++i){
			Eigen::Vector3d currP = points[i];
			Eigen::Vector3d direction1 = startP - currP;
			Eigen::Vector3d direction2 = endP - currP;
			double currAngle = trajPlanner::angleBetweenVectors(direction1, direction2);
			if (currAngle < minAngle){
				minAngle = currAngle;
			}
		}
		return minAngle;
	}

	void dynamicInspection::getSideWallPoints(const std::vector<std::vector<Eigen::Vector3d>>& potentialWallVec, std::vector<Eigen::Vector3d>& finalSideWallPoints){
		double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		Eigen::Vector3d currDirection (cos(currYaw), sin(currYaw), 0.0);
		Eigen::Vector3d currP (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);

		double minAngle = PI_const * 2;
		size_t minIdx = 0;
		for (size_t i=0; i<potentialWallVec.size(); ++i){
			for (size_t j=0; j<potentialWallVec[i].size(); ++j){
				Eigen::Vector3d pointDirection = potentialWallVec[i][j] - currDirection;
				double angle = trajPlanner::angleBetweenVectors(currDirection, pointDirection);
				if (angle < minAngle){
					minAngle = angle;
					minIdx = i;
				}
			}
		}

		if (potentialWallVec.size() != 0){
			finalSideWallPoints = potentialWallVec[minIdx];
			// check the order of the wall points
			// defined order: the last point is the newly discovered point
			Eigen::Vector3d firstPoint = finalSideWallPoints[0];
			Eigen::Vector3d firstDirection = firstPoint - currP;
			Eigen::Vector3d lastPoint = finalSideWallPoints.back();
			Eigen::Vector3d lastDirection = lastPoint - currP;
			double firstPointAngle = trajPlanner::angleBetweenVectors(currDirection, firstDirection);
			double lastPointAngle =  trajPlanner::angleBetweenVectors(currDirection, lastDirection);

			if (lastPointAngle > firstPointAngle){
				std::reverse(finalSideWallPoints.begin(), finalSideWallPoints.end());
			}

		}


		this->finalSideWallPoints_ = finalSideWallPoints;

		std::vector<Eigen::Vector3d> finalSideWallPointsC;
		for (Eigen::Vector3d wallPoint : this->finalSideWallPoints_){
			Eigen::Vector3d wallPointC = wallPoint - currP;
			finalSideWallPointsC.push_back(wallPointC);
		}
		this->finalSideWallPointsC_ = finalSideWallPointsC;
	}	


	void dynamicInspection::getSideWallRaycastMsg(sensor_msgs::ImagePtr& imgMsg){
		// draw sensor range
		double imgRes = 0.1;
		int imageSize = int(this->sideWallRaycastRange_/imgRes) * 2 + 10;
		cv::Mat img (imageSize, imageSize, CV_8UC3, Scalar(255, 255, 255));
		Point center (imageSize/2, imageSize/2);
		int radius = int(this->sideWallRaycastRange_/imgRes);
		cv::Scalar circleColor(255, 0, 0);//Color of the circle
		int thickness = 1;
		cv::circle(img, center, radius, circleColor, thickness);

		// draw current position
		cv::Scalar posColor(0, 255, 0);//Color of the circle
		cv::circle(img, center, 1, posColor, 1);

		// visualize the raycast points
		for (size_t i=0; i<this->sideWallPointsC_.size(); ++i){
			int locX = int(this->sideWallPointsC_[i](0)/imgRes + imageSize/2);
			int locY = int(this->sideWallPointsC_[i](1)/imgRes + imageSize/2);
			Point pCenter (locY, locX);
			int pRadius = 1;
			cv::Scalar pColor(0, 0, 255);//Color of the circle
			int pThickness = 1;
			cv::circle(img, pCenter, pRadius, pColor, pThickness);
		}

		// visualize the wall with line
		if (this->finalSideWallPointsC_.size() != 0){

			for (size_t i=0; i<this->finalSideWallPointsC_.size()-1; ++i){
				Eigen::Vector3d p1e = this->finalSideWallPointsC_[i];
				Eigen::Vector3d p2e = this->finalSideWallPointsC_[i+1];
				cv::Point p1 (int(p1e(1)/imgRes + imageSize/2), int(p1e(0)/imgRes + imageSize/2));
				cv::Point p2 (int(p2e(1)/imgRes + imageSize/2), int(p2e(0)/imgRes + imageSize/2));
				cv::line(img, p1, p2, Scalar(255, 0, 120), 1, LINE_8);
			}
		}

		cv::flip(img, img, -1);
		imgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
	}


	geometry_msgs::PoseStamped dynamicInspection::eigen2ps(const Eigen::Vector3d& p){
		geometry_msgs::PoseStamped ps;
		ps.pose.position.x = p(0);
		ps.pose.position.y = p(1);
		ps.pose.position.z = p(2);
		return ps;
	}
}
