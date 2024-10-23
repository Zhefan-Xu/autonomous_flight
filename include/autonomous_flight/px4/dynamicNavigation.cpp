/*
	FILE: dynamicNavigation.cpp
	------------------------
	dynamic navigation implementation file
*/
#include <autonomous_flight/px4/dynamicNavigation.h>

namespace AutoFlight{
	dynamicNavigation::dynamicNavigation(const ros::NodeHandle& nh) : flightBase(nh){
		this->initParam();
		this->initModules();
		this->registerPub();
		if (this->useFakeDetector_){
			// free map callback
			this->freeMapTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dynamicNavigation::freeMapCB, this);
		}
		
	}

	void dynamicNavigation::initParam(){
    	// parameters    
    	// use simulation detector	
		if (not this->nh_.getParam("autonomous_flight/use_fake_detector", this->useFakeDetector_)){
			this->useFakeDetector_ = false;
			cout << "[AutoFlight]: No use fake detector param found. Use default: false." << endl;
		}
		else{
			cout << "[AutoFlight]: Use fake detector is set to: " << this->useFakeDetector_ << "." << endl;
		}

		// use predictor	
		if (not this->nh_.getParam("autonomous_flight/use_predictor", this->usePredictor_)){
			this->usePredictor_ = false;
			cout << "[AutoFlight]: No use predictor param found. Use default: false." << endl;
		}
		else{
			cout << "[AutoFlight]: Use predictor is set to: " << this->usePredictor_ << "." << endl;
		}


    	// use global planner or not	
		if (not this->nh_.getParam("autonomous_flight/use_global_planner", this->useGlobalPlanner_)){
			this->useGlobalPlanner_ = false;
			cout << "[AutoFlight]: No use global planner param found. Use default: false." << endl;
		}
		else{
			cout << "[AutoFlight]: Global planner use is set to: " << this->useGlobalPlanner_ << "." << endl;
		}

		//Use B-spline Planner
		if (not this->nh_.getParam("autonomous_flight/use_bspline_planner", this->useBsplinePlanner_)){
			this->useBsplinePlanner_ = true;
			cout << "[AutoFlight]: No use B-spline planner param found. Use default: true." << endl;
		}
		else{
			cout << "[AutoFlight]: Use B-spline planner is set to: " << this->useBsplinePlanner_ << "." << endl;
		}

		//Use MPC Planner
		if (not this->nh_.getParam("autonomous_flight/use_mpc_planner", this->useMPCPlanner_)){
			this->useMPCPlanner_ = false;
			cout << "[AutoFlight]: No use MPC planner param found. Use default: false." << endl;
		}
		else{
			cout << "[AutoFlight]: Use MPC planner is set to: " << this->useMPCPlanner_ << "." << endl;
		}	

		if (this->useMPCPlanner_ and not this->useBsplinePlanner_){
			this->plannerType_ = PLANNER::MPC;
		}
		else if (this->useBsplinePlanner_ and this->useMPCPlanner_){
			this->plannerType_ = PLANNER::MIXED;
		}
		else{
			this->plannerType_ = PLANNER::BSPLINE;
		}

		// No turning of yaw
		if (not this->nh_.getParam("autonomous_flight/no_yaw_turning", this->noYawTurning_)){
			this->noYawTurning_ = false;
			cout << "[AutoFlight]: No yaw turning param found. Use default: false." << endl;
		}
		else{
			cout << "[AutoFlight]: Yaw turning use is set to: " << this->noYawTurning_ << "." << endl;
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

		// desired acceleration
		if (not this->nh_.getParam("autonomous_flight/desired_acceleration", this->desiredAcc_)){
			this->desiredAcc_ = 1.0;
			cout << "[AutoFlight]: No desired acceleration param found. Use default: 1.0 m/s^2." << endl;
		}
		else{
			cout << "[AutoFlight]: Desired acceleration is set to: " << this->desiredAcc_ << "m/s^2." << endl;
		}

    	// desired angular velocity    	
		if (not this->nh_.getParam("autonomous_flight/desired_angular_velocity", this->desiredAngularVel_)){
			this->desiredAngularVel_ = 1.0;
			cout << "[AutoFlight]: No desired angular velocity param found. Use default: 0.5 rad/s." << endl;
		}
		else{
			cout << "[AutoFlight]: Desired angular velocity is set to: " << this->desiredAngularVel_ << "rad/s." << endl;
		}	

    	// replan time for dynamic obstacle
		if (not this->nh_.getParam("autonomous_flight/replan_time_for_dynamic_obstacles", this->replanTimeForDynamicObstacle_)){
			this->replanTimeForDynamicObstacle_ = 0.3;
			cout << "[AutoFlight]: No dynamic obstacle replan time param found. Use default: 0.3s." << endl;
		}
		else{
			cout << "[AutoFlight]: Dynamic obstacle replan time is set to: " << this->replanTimeForDynamicObstacle_ << "s." << endl;
		}	

    	// trajectory data save path   	
		if (not this->nh_.getParam("autonomous_flight/trajectory_info_save_path", this->trajSavePath_)){
			this->trajSavePath_ = "No";
			cout << "[AutoFlight]: No trajectory info save path param found. Use current directory." << endl;
		}
		else{
			cout << "[AutoFlight]: Trajectory info save path is set to: " << this->trajSavePath_ << "." << endl;
		}	

		// whether or not to use predefined goal
		if (not this->nh_.getParam("autonomous_flight/use_predefined_goal", this->usePredefinedGoal_)){
			this->usePredefinedGoal_ = false;
			cout << "[AutoFlight]: No use predefined goal param found. Use default: false." << endl;
		} 
		else{
			cout << "[AutoFlight]: Use predefined goal is set to: " << this->usePredefinedGoal_ << "." << endl;
		}

		// predefined goal parameter
		std::vector<double> goalVecTemp;
		if (not this->nh_.getParam("autonomous_flight/goal", goalVecTemp)){
			this->predefinedGoal_.poses.clear();
			cout << "[AutoFlight]: No use predefined goal param found. Use default: false." << endl;
		} 
		else{
			int numGoals = int(goalVecTemp.size())/3;
			std::vector<geometry_msgs::PoseStamped> pathTemp;
			for (int i=0; i<numGoals; ++i){
				geometry_msgs::PoseStamped goal;
				goal.pose.position.x = goalVecTemp[i*3+0];
				goal.pose.position.y = goalVecTemp[i*3+1];
				goal.pose.position.z = goalVecTemp[i*3+2];
				pathTemp.push_back(goal);
				cout << "[AutoFlight]: Goal is set to: " << goal.pose.position.x <<", "<< goal.pose.position.y<<", "<< goal.pose.position.z << "." << endl;
			}
			this->predefinedGoal_.poses = pathTemp;
		}

		// whether or not to repeat tracking predefined path
		if (not this->nh_.getParam("autonomous_flight/execute_path_times", this->repeatPathNum_)){
			this->repeatPathNum_ = 1;
			cout << "[AutoFlight]: No execute path number of times param found. Use default: 1." << endl;
		} 
		else{
			cout << "[AutoFlight]: Execute path number of times is set to: " << this->repeatPathNum_ << "." << endl;
		}		
	}

	void dynamicNavigation::initModules(){
		// initialize map
		if (this->useFakeDetector_){
			// initialize fake detector
			this->detector_.reset(new onboardDetector::fakeDetector (this->nh_));	
			this->map_.reset(new mapManager::dynamicMap (this->nh_, false));
		}
		else{
			this->map_.reset(new mapManager::dynamicMap (this->nh_));
		}

		// initialize predictor
		if (this->usePredictor_){
			this->predictor_.reset(new dynamicPredictor::predictor (this->nh_));
			this->predictor_->setMap(this->map_);
			if (this->useFakeDetector_){
				this->predictor_->setDetector(this->detector_);
			}
		}

		// initialize rrt planner
		this->rrtPlanner_.reset(new globalPlanner::rrtOccMap<3> (this->nh_));
		this->rrtPlanner_->setMap(this->map_);

		// initialize polynomial trajectory planner
		this->polyTraj_.reset(new trajPlanner::polyTrajOccMap (this->nh_));
		this->polyTraj_->setMap(this->map_);
		this->polyTraj_->updateDesiredVel(this->desiredVel_);
		this->polyTraj_->updateDesiredAcc(this->desiredAcc_);

		// initialize piecewise linear trajectory planner
		this->pwlTraj_.reset(new trajPlanner::pwlTraj (this->nh_));

		if (this->useMPCPlanner_){
			this->mpc_.reset(new trajPlanner::mpcPlanner (this->nh_));
			this->mpc_->updateMaxVel(this->desiredVel_*1.5);
			this->mpc_->updateMaxAcc(this->desiredAcc_);
			this->mpc_->setMap(this->map_);
		}
		if (this->useBsplinePlanner_){
			// initialize bspline trajectory planner
			this->bsplineTraj_.reset(new trajPlanner::bsplineTraj (this->nh_));
			this->bsplineTraj_->setMap(this->map_);
			this->bsplineTraj_->updateMaxVel(this->desiredVel_);
			this->bsplineTraj_->updateMaxAcc(this->desiredAcc_);
		}
	}

	void dynamicNavigation::registerPub(){
		this->rrtPathPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicNavigation/rrt_path", 10);
		this->polyTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicNavigation/poly_traj", 10);
		this->pwlTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicNavigation/pwl_trajectory", 10);
		if (this->useMPCPlanner_){
			this->mpcTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicNavigation/mpc_trajectory", 10);
		}
		if (this->useBsplinePlanner_){
			this->bsplineTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicNavigation/bspline_trajectory", 10);
		}
		this->inputTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicNavigation/input_trajectory", 10);
		this->goalPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("dynamicNavigation/goal", 10);
	}

	void dynamicNavigation::registerCallback(){
		if (this->useMPCPlanner_){
			this->mpcWorker_ = std::thread(&dynamicNavigation::mpcCB, this);
			this->mpcWorker_.detach();
			// this->mpcTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dynamicNavigation::mpcCB, this);
		}
		if (this->useBsplinePlanner_){
			// planner callback
			this->plannerTimer_ = this->nh_.createTimer(ros::Duration(0.02), &dynamicNavigation::plannerCB, this);
		}

		// collision check callback
		this->replanCheckTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dynamicNavigation::replanCheckCB, this);

		// trajectory execution callback
		this->trajExeTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dynamicNavigation::trajExeCB, this);

		// visualization callback
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.033), &dynamicNavigation::visCB, this);
	}

	void dynamicNavigation::mpcCB(){
		ros::Rate r(10);
		while (ros::ok()){
			if (this->mpcReplan_){
				this->replanning_ = true;
				if (not this->refTrajReady_){
					if (this->plannerType_ == PLANNER::MPC){
						if (this->usePredefinedGoal_){			
							Eigen::Vector3d startVel (0, 0, 0);
							Eigen::Vector3d startAcc (0, 0, 0);
							Eigen::Vector3d endVel (0, 0, 0);
							Eigen::Vector3d endAcc (0, 0, 0);
							std::vector<Eigen::Vector3d> startEndConditions {startVel, startAcc, endVel, endAcc};

							this->polyTraj_->updatePath(this->predefinedGoal_, startEndConditions);
							this->polyTraj_->makePlan(this->polyTrajMsg_); // include corridor constraint

							double dt = 0.1; 
							nav_msgs::Path mpcInputTraj = this->polyTraj_->getTrajectory(dt);
							this->mpc_->updatePath(mpcInputTraj, dt);
							this->inputTrajMsg_ = mpcInputTraj;
							this->mpcFirstTime_ = true;
							this->repeatPathNum_ -= 1;
							this->refTrajReady_ = true;
							this->trackingStartTime_ = ros::Time::now();
						}
						else{
							if (this->useGlobalPlanner_){
								this->rrtPlanner_->updateStart(this->odom_.pose.pose);
								this->rrtPlanner_->updateGoal(this->goal_.pose);
								nav_msgs::Path rrtPathMsgTemp;
								this->rrtPlanner_->makePlan(rrtPathMsgTemp);
								if (rrtPathMsgTemp.poses.size() >= 2){
									this->rrtPathMsg_ = rrtPathMsgTemp;
								}
								Eigen::Vector3d startVel (0, 0, 0);
								Eigen::Vector3d startAcc (0, 0, 0);
								Eigen::Vector3d endVel (0, 0, 0);
								Eigen::Vector3d endAcc (0, 0, 0);
								std::vector<Eigen::Vector3d> startEndConditions {startVel, startAcc, endVel, endAcc};

								this->polyTraj_->updatePath(rrtPathMsgTemp, startEndConditions);
								this->polyTraj_->makePlan(this->polyTrajMsg_); // include corridor constraint		
								
								
								double dt = 0.1;
								nav_msgs::Path mpcInputTraj = this->polyTraj_->getTrajectory(dt);
								this->mpc_->updatePath(mpcInputTraj,dt);
								this->inputTrajMsg_ = mpcInputTraj;
								this->refTrajReady_ = true;
								this->mpcFirstTime_ = true;
								this->trackingStartTime_ = ros::Time::now();
							}
							else{
								nav_msgs::Path waypoints, polyTrajTemp;
								geometry_msgs::PoseStamped start, goal;
								start.pose = this->odom_.pose.pose; goal = this->goal_;
								waypoints.poses = std::vector<geometry_msgs::PoseStamped> {start, goal};					
								
								Eigen::Vector3d startVel (0, 0, 0);
								Eigen::Vector3d startAcc (0, 0, 0);
								Eigen::Vector3d endVel (0, 0, 0);
								Eigen::Vector3d endAcc (0, 0, 0);
								std::vector<Eigen::Vector3d> startEndConditions {startVel, startAcc, endVel, endAcc};

								this->polyTraj_->updatePath(waypoints, startEndConditions);
								this->polyTraj_->makePlan(this->polyTrajMsg_); // include corridor constraint

								double dt = 0.1;
								nav_msgs::Path mpcInputTraj = this->polyTraj_->getTrajectory(dt);
								this->mpc_->updatePath(mpcInputTraj,dt);
								this->inputTrajMsg_ = mpcInputTraj;
								this->refTrajReady_ = true;
								this->mpcFirstTime_ = true;
								this->trackingStartTime_ = ros::Time::now();
							}
						}
					}
					else if (this->plannerType_ == PLANNER::MIXED){
						if (this->bsplineTrajectoryReady_){
							double dt = 0.1;
							nav_msgs::Path mpcInputTraj = this->bsplineTrajMsg_;
							this->mpc_->updatePath(mpcInputTraj,dt);
							// this->inputTrajMsg_ = mpcInputTraj;
							this->refTrajReady_ = true;
							this->mpcFirstTime_ = true;
							this->trackingStartTime_ = ros::Time::now();
						}
						else{
							this->stop();
						}
					}
				}
				else if (this->refTrajReady_){
					Eigen::Vector3d currPos = this->currPos_;
					Eigen::Vector3d currVel = this->currVel_;
	
					this->mpc_->updateCurrStates(currPos, currVel);
					if (this->usePredictor_){
						std::vector<std::vector<std::vector<Eigen::Vector3d>>> predPos, predSize;
						std::vector<Eigen::VectorXd> intentProb;
						this->predictor_->getPrediction(predPos, predSize, intentProb);
						this->mpc_->updatePredObstacles(predPos, predSize, intentProb);
					}
					else{
						std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
						if (this->useFakeDetector_){
							Eigen::Vector3d robotSize;
							this->map_->getRobotSize(robotSize);
							this->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize, robotSize);
						}
						else{ 
							this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
						}
						this->mpc_->updateDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
					}

					ros::Time trajStartTime = ros::Time::now();
					bool newTrajReturn;
					if (this->usePredictor_){
						// makePlan with predictor
						newTrajReturn = this->mpc_->makePlanWithPred();
					}
					else{
						newTrajReturn = this->mpc_->makePlan();
					}
					nav_msgs::Path mpcTraj;	
					
					if (newTrajReturn){
						this->trajStartTime_ = trajStartTime;
						if (this->mpcHasCollision() or this->hasDynamicCollision()){
							this->mpcTrajectoryReady_ = false;
							this->stop();
						}
						else{
							this->mpc_->getTrajectory(mpcTraj);
							this->mpcTrajMsg_ = mpcTraj;
							this->mpcTrajectoryReady_ = true;
							this->mpcFirstTime_ = false;
						}
					}
					else if (not this->mpcFirstTime_){
						if (this->mpcHasCollision() or this->hasDynamicCollision()){
							this->mpcTrajectoryReady_ = false;
							this->stop();
						}
						else{
							this->mpcTrajectoryReady_ = true;
						}
					}
					else{
						this->mpcTrajectoryReady_ = false;
						this->stop();	
					}
				}
			}
			this->replanning_ = false;
			r.sleep();
		}
	}

	void dynamicNavigation::plannerCB(const ros::TimerEvent&){
		if (not this->firstGoal_) return;

		if (this->bsplineReplan_){
			std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
			if (this->useFakeDetector_){
				Eigen::Vector3d robotSize;
				this->map_->getRobotSize(robotSize);
				this->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize, robotSize);
			}
			else{ 
				this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}
			// get start and end condition for trajectory generation (the end condition is the final zero condition)
			std::vector<Eigen::Vector3d> startEndConditions;
			this->getStartEndConditions(startEndConditions); 
			nav_msgs::Path inputTraj;
			// bspline trajectory generation
			double finalTime; // final time for bspline trajectory
			double initTs = this->bsplineTraj_->getInitTs();
			if (this->useGlobalPlanner_){
				if (this->needGlobalPlan_){
					this->rrtPlanner_->updateStart(this->odom_.pose.pose);
					this->rrtPlanner_->updateGoal(this->goal_.pose);
					nav_msgs::Path rrtPathMsgTemp;
					this->rrtPlanner_->makePlan(rrtPathMsgTemp);
					if (rrtPathMsgTemp.poses.size() >= 2){
						this->rrtPathMsg_ = rrtPathMsgTemp;
						this->globalPlanReady_ = true;
					}
					this->needGlobalPlan_ = false;
					return;
				}
				else{
					if (this->globalPlanReady_){
						// get rest of global plan
						nav_msgs::Path restPath = this->getRestGlobalPath();
						this->polyTraj_->updatePath(restPath, startEndConditions);
						this->polyTraj_->makePlan(this->polyTrajMsg_); // no corridor constraint		
						nav_msgs::Path adjustedInputPolyTraj;
						bool satisfyDistanceCheck = false;
						double dtTemp = initTs;
						double finalTimeTemp;
						ros::Time startTime = ros::Time::now();
						ros::Time currTime;
						while (ros::ok()){
							currTime = ros::Time::now();
							if ((currTime - startTime).toSec() >= 0.05){
								cout << "[AutoFlight]: Exceed path check time. Use the best." << endl;
								break;
							}
							nav_msgs::Path inputPolyTraj = this->polyTraj_->getTrajectory(dtTemp);
							satisfyDistanceCheck = this->bsplineTraj_->inputPathCheck(inputPolyTraj, adjustedInputPolyTraj, dtTemp, finalTimeTemp);
							if (satisfyDistanceCheck) break;
							dtTemp *= 0.8;
						}

						inputTraj = adjustedInputPolyTraj;
						finalTime = finalTimeTemp;
						startEndConditions[1] = this->polyTraj_->getVel(finalTime);
						startEndConditions[3] = this->polyTraj_->getAcc(finalTime);	

					}
					else{
						cout << "[AutoFlight]: Global planner fails. Check goal and map." << endl;
					}		
				}				
			}
			else{
				if (obstaclesPos.size() == 0 or this->plannerType_ == PLANNER::MIXED){ // use prev planned trajectory if there is no dynamic obstacle
					if (not this->bsplineTrajectoryReady_){ // use polynomial trajectory as input
						nav_msgs::Path waypoints, polyTrajTemp;
						geometry_msgs::PoseStamped start, goal;
						start.pose = this->odom_.pose.pose; goal = this->goal_;
						waypoints.poses = std::vector<geometry_msgs::PoseStamped> {start, goal};					
						
						this->polyTraj_->updatePath(waypoints, startEndConditions);
						this->polyTraj_->makePlan(false); // no corridor constraint
						
						nav_msgs::Path adjustedInputPolyTraj;
						bool satisfyDistanceCheck = false;
						double dtTemp = initTs;
						double finalTimeTemp;
						ros::Time startTime = ros::Time::now();
						ros::Time currTime;
						while (ros::ok()){
							currTime = ros::Time::now();
							if ((currTime - startTime).toSec() >= 0.05){
								cout << "[AutoFlight]: Exceed path check time. Use the best." << endl;
								break;
							}
							nav_msgs::Path inputPolyTraj = this->polyTraj_->getTrajectory(dtTemp);
							satisfyDistanceCheck = this->bsplineTraj_->inputPathCheck(inputPolyTraj, adjustedInputPolyTraj, dtTemp, finalTimeTemp);
							if (satisfyDistanceCheck) break;
							
							dtTemp *= 0.8;
						}

						inputTraj = adjustedInputPolyTraj;
						finalTime = finalTimeTemp;
						startEndConditions[1] = this->polyTraj_->getVel(finalTime);
						startEndConditions[3] = this->polyTraj_->getAcc(finalTime);
					}
					else{
						Eigen::Vector3d bsplineLastPos = this->trajectory_.at(this->trajectory_.getDuration());
						geometry_msgs::PoseStamped lastPs; lastPs.pose.position.x = bsplineLastPos(0); lastPs.pose.position.y = bsplineLastPos(1); lastPs.pose.position.z = bsplineLastPos(2);
						Eigen::Vector3d goalPos (this->goal_.pose.position.x, this->goal_.pose.position.y, this->goal_.pose.position.z);
						// check the distance between last point and the goal position
						if ((bsplineLastPos - goalPos).norm() >= 0.2){ // use polynomial trajectory to make the rest of the trajectory
							nav_msgs::Path waypoints, polyTrajTemp;
							waypoints.poses = std::vector<geometry_msgs::PoseStamped>{lastPs, this->goal_};
							std::vector<Eigen::Vector3d> polyStartEndConditions;
							Eigen::Vector3d polyStartVel = this->trajectory_.getDerivative().at(this->trajectory_.getDuration());
							Eigen::Vector3d polyEndVel (0.0, 0.0, 0.0);
							Eigen::Vector3d polyStartAcc = this->trajectory_.getDerivative().getDerivative().at(this->trajectory_.getDuration());
							Eigen::Vector3d polyEndAcc (0.0, 0.0, 0.0);
							polyStartEndConditions.push_back(polyStartVel);
							polyStartEndConditions.push_back(polyEndVel);
							polyStartEndConditions.push_back(polyStartAcc);
							polyStartEndConditions.push_back(polyEndAcc);
							this->polyTraj_->updatePath(waypoints, polyStartEndConditions);
							this->polyTraj_->makePlan(false); // no corridor constraint
							
							nav_msgs::Path adjustedInputCombinedTraj;
							bool satisfyDistanceCheck = false;
							double dtTemp = initTs;
							double finalTimeTemp;
							ros::Time startTime = ros::Time::now();
							ros::Time currTime;
							while (ros::ok()){
								currTime = ros::Time::now();
								if ((currTime - startTime).toSec() >= 0.05){
									cout << "[AutoFlight]: Exceed path check time. Use the best." << endl;
									break;
								}							
								nav_msgs::Path inputRestTraj = this->getCurrentTraj(dtTemp);
								nav_msgs::Path inputPolyTraj = this->polyTraj_->getTrajectory(dtTemp);
								nav_msgs::Path inputCombinedTraj;
								inputCombinedTraj.poses = inputRestTraj.poses;
								for (size_t i=1; i<inputPolyTraj.poses.size(); ++i){
									inputCombinedTraj.poses.push_back(inputPolyTraj.poses[i]);
								}
								
								satisfyDistanceCheck = this->bsplineTraj_->inputPathCheck(inputCombinedTraj, adjustedInputCombinedTraj, dtTemp, finalTimeTemp);
								if (satisfyDistanceCheck) break;
								
								dtTemp *= 0.8; // magic number 0.8
							}
							inputTraj = adjustedInputCombinedTraj;
							finalTime = finalTimeTemp - this->trajectory_.getDuration(); // need to subtract prev time since it is combined trajectory
							startEndConditions[1] = this->polyTraj_->getVel(finalTime);
							startEndConditions[3] = this->polyTraj_->getAcc(finalTime);
						}
						else{
							nav_msgs::Path adjustedInputRestTraj;
							bool satisfyDistanceCheck = false;
							double dtTemp = initTs;
							double finalTimeTemp;
							ros::Time startTime = ros::Time::now();
							ros::Time currTime;
							while (ros::ok()){
								currTime = ros::Time::now();
								if ((currTime - startTime).toSec() >= 0.05){
									cout << "[AutoFlight]: Exceed path check time. Use the best." << endl;
									break;
								}
								nav_msgs::Path inputRestTraj = this->getCurrentTraj(dtTemp);
								satisfyDistanceCheck = this->bsplineTraj_->inputPathCheck(inputRestTraj, adjustedInputRestTraj, dtTemp, finalTimeTemp);
								if (satisfyDistanceCheck) break;
								
								dtTemp *= 0.8;
							}
							inputTraj = adjustedInputRestTraj;
						}
					}
				}
				else{
					nav_msgs::Path simplePath;
					geometry_msgs::PoseStamped pStart, pGoal;
					pStart.pose = this->odom_.pose.pose;
					pGoal = this->goal_;
					std::vector<geometry_msgs::PoseStamped> pathVec {pStart, pGoal};
					simplePath.poses = pathVec;				
					this->pwlTraj_->updatePath(simplePath, 1.0, false);
					this->pwlTraj_->makePlan(inputTraj, this->bsplineTraj_->getControlPointDist());
				}
			}
			

			this->inputTrajMsg_ = inputTraj;

			bool updateSuccess = this->bsplineTraj_->updatePath(inputTraj, startEndConditions);
			if (obstaclesPos.size() != 0 and updateSuccess and PLANNER::BSPLINE){
				this->bsplineTraj_->updateDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}
			if (updateSuccess){
				nav_msgs::Path bsplineTrajMsgTemp;
				bool planSuccess = this->bsplineTraj_->makePlan(bsplineTrajMsgTemp);
				if (planSuccess){
					this->bsplineTrajMsg_ = bsplineTrajMsgTemp;
					// this->trajStartTime_ = ros::Time::now();
					this->trajTime_ = 0.0; // reset trajectory time
					this->trajectory_ = this->bsplineTraj_->getTrajectory();
					this->bsplineTrajectoryReady_ = true;
					this->bsplineReplan_ = false;
					cout << "\033[1;32m[AutoFlight]: Trajectory generated successfully.\033[0m " << endl;
				}
				else{
					// if the current trajectory is still valid, then just ignore this iteration
					// if the current trajectory/or new goal point is assigned is not valid, then just stop
					if (this->bsplineHasCollision()){
						this->bsplineTrajectoryReady_ = false;
						this->refTrajReady_ = false;
						this->stop();
						cout << "[AutoFlight]: Stop!!! Trajectory generation fails." << endl;
						this->bsplineReplan_ = false;
						if (this->plannerType_ == PLANNER::MIXED){
							this->mpcReplan_ = false;
							this->mpcTrajectoryReady_ = false;
						}
					}
					else{
						if (this->bsplineTrajectoryReady_){
							cout << "[AutoFlight]: Trajectory fail. Use trajectory from previous iteration." << endl;
							this->bsplineReplan_ = false;
						}
						else{
							cout << "[AutoFlight]: Unable to generate a feasible trajectory. Please provide a new goal." << endl;
							this->bsplineReplan_ = false;
							if (this->plannerType_ == PLANNER::MIXED){
								this->mpcReplan_ = false;
								this->mpcTrajectoryReady_ = false;
							}
						}
					}
				}
			}
			else{
				this->bsplineTrajectoryReady_ = false;
				this->refTrajReady_ = false;
				this->stop();
				this->bsplineReplan_ = false;
				cout << "[AutoFlight]: Goal is not valid. Stop." << endl;
			}
		}
	}

	void dynamicNavigation::replanCheckCB(const ros::TimerEvent&){
		/*
			Replan if
			1. collision detected
			2. new goal point assigned
			3. fixed distance
		*/
		if (this->plannerType_ == PLANNER::MPC){
			if (this->usePredefinedGoal_){
				if (not this->refTrajReady_ and this->predefinedGoal_.poses.size()>0){
					this->mpcReplan_ = false;
					this->refTrajReady_ = false;
					this->goal_ = this->predefinedGoal_.poses.back();
					this->mpcReplan_ = true;
					// cout<<"[AutoFlight]: Plan for reference Trajectory using pre-defined goal"<<endl;
					return;
				}
			}
			else if (this->goalReceived_){
				this->mpcReplan_ = false;
				this->mpcTrajectoryReady_ = false;
				ros::Rate r(200);
				while(ros::ok() and this->replanning_){
					r.sleep();
				}
				this->mpcTrajectoryReady_ = false;
				if(this->goalHasCollision()){
					this->refTrajReady_ = false;
					this->goalReceived_ = false;
					cout << "[AutoFlight]: Invalid goal position, please assign a new goal." << endl; 
					return;
				}
				else{
					this->refTrajReady_ = false;
					if (not this->noYawTurning_ ){
						double yaw = atan2(this->goal_.pose.position.y - this->odom_.pose.pose.position.y, this->goal_.pose.position.x - this->odom_.pose.pose.position.x);
						this->facingYaw_ = yaw;
						this->moveToOrientation(yaw, this->desiredAngularVel_);
					}
					this->firstTimeSave_ = true;
					this->mpcReplan_ = true;
					this->goalReceived_ = false;
					if (this->useGlobalPlanner_){
						cout << "[AutoFlight]: Start global planning." << endl;
					}

					cout << "[AutoFlight]: Replan for new goal position." << endl; 
					return;
				}
			}
			if (this->mpcTrajectoryReady_){
				if (this->usePredefinedGoal_){
					ros::Time currTime = ros::Time::now();
					if (this->mpcHasCollision() or this->hasDynamicCollision()){ 
						this->stop();
						this->mpcTrajectoryReady_ = false;
						this->mpcReplan_ = true;
						cout << "[AutoFlight]: Collision detected. MPC replan." << endl;
						return;
					}
					else if (AutoFlight::getPoseDistance(this->odom_.pose.pose, this->goal_.pose) <= 0.3 and 
						(currTime-this->trackingStartTime_ ).toSec() >= 3){
						if (this->repeatPathNum_ == 0){
							this->mpcReplan_ = false;
							this->mpcTrajectoryReady_ = false;
							ros::Rate r(200);
							while(ros::ok() and this->replanning_){
								r.sleep();
							}
							this->mpcTrajectoryReady_ = false;
							this->stop();
							this->predefinedGoal_.poses.clear();
							this->refTrajReady_ = false;
							cout << "[AutoFlight]: Goal reached. MPC Stop replan." << endl;
						}
						else{
							double dt = 0.1;
							nav_msgs::Path mpcInputTraj = this->polyTraj_->getTrajectory(dt);
							this->mpc_->updatePath(mpcInputTraj, dt);
							this->inputTrajMsg_ = mpcInputTraj;
							this->mpcFirstTime_ = true;
							if (this->repeatPathNum_ > 1){
								cout << "[AutoFlight]: Goal reached. " << this->repeatPathNum_ << " rounds left." << endl;
							}
							else{
								cout << "[AutoFlight]: Goal reached. " << this->repeatPathNum_ << " round left." << endl;
							}
							this->repeatPathNum_ -= 1;
							this->refTrajReady_ = true;
							this->trackingStartTime_ = ros::Time::now();
							this->mpcReplan_ = true;
						}
						return;
					}

				}
				else{					
					if (this->goalHasCollision()){
						this->mpcReplan_ = false;
						this->mpcTrajectoryReady_ = false;
						ros::Rate r(200);
						while(ros::ok() and this->replanning_){
							r.sleep();
						}
						this->mpcTrajectoryReady_ = false;
						this->stop();
						this->refTrajReady_ = false;
						this->mpcFirstTime_ = true;
						cout<<"[AutoFlight]: Invalid goal. Stop!" << endl;
						return;
					}
					else if (this->mpcHasCollision() or this->hasDynamicCollision()){ 
						this->stop();
						this->mpcTrajectoryReady_ = false;
						this->mpcReplan_ = true;
						cout << "[AutoFlight]: Collision detected. MPC replan." << endl;
						return;
					}
					else if(AutoFlight::getPoseDistance(this->odom_.pose.pose, this->goal_.pose) <= 0.3){
						this->mpcReplan_ = false;
						this->mpcTrajectoryReady_ = false;
						ros::Rate r(200);
						while(ros::ok() and this->replanning_){
							r.sleep();
						}
						this->stop();
						this->refTrajReady_ = false;
						this->mpcTrajectoryReady_ = false;
						this->mpcFirstTime_ = true;
						cout << "[AutoFlight]: Goal reached. MPC Stop replan." << endl;
						return;
					}
				}
			}
		}
		else if (this->plannerType_ == PLANNER::BSPLINE){
			if (this->goalReceived_){
				this->bsplineReplan_ = false;
				this->bsplineTrajectoryReady_ = false;
				if (not this->noYawTurning_ and not this->useYawControl_){
					double yaw = atan2(this->goal_.pose.position.y - this->odom_.pose.pose.position.y, this->goal_.pose.position.x - this->odom_.pose.pose.position.x);
					this->facingYaw_ = yaw;
					this->moveToOrientation(yaw, this->desiredAngularVel_);
				}
				this->firstTimeSave_ = true;
				this->bsplineReplan_ = true;
				this->goalReceived_ = false;
				if (this->useGlobalPlanner_){
					cout << "[AutoFlight]: Start global planning." << endl;
					this->needGlobalPlan_ = true;
					this->globalPlanReady_ = false;
				}

				cout << "[AutoFlight]: Replan for new goal position." << endl; 
				return;
			}

			if (this->bsplineTrajectoryReady_){
				if (this->bsplineHasCollision()){ // if trajectory not ready, do not replan
					this->bsplineReplan_ = true;
					cout << "[AutoFlight]: Replan for collision." << endl;
					return;
				}

				// replan for dynamic obstacles
				if (this->computeExecutionDistance() >= 0.3 and this->hasDynamicCollision()){
				// if (this->hasDynamicObstacle()){
					this->bsplineReplan_ = true;
					cout << "[AutoFlight]: Replan for dynamic obstacles." << endl;
					return;
				}

				if (this->computeExecutionDistance() >= 1.5 and AutoFlight::getPoseDistance(this->odom_.pose.pose, this->goal_.pose) >= 3){
					this->bsplineReplan_ = true;
					cout << "[AutoFlight]: Regular replan." << endl;
					return;
				}

				if (this->computeExecutionDistance() >= 0.3 and this->replanForDynamicObstacle()){
					this->bsplineReplan_ = true;
					cout << "[AutoFlight]: Regular replan for dynamic obstacles." << endl;
					return;
				}
			}
		}
		else if (this->plannerType_ == PLANNER::MIXED){
			if (this->goalReceived_){
				this->bsplineReplan_ = false;
				this->mpcReplan_ = false;
				this->bsplineTrajectoryReady_ = false;
				this->mpcTrajectoryReady_ = false;
				if (not this->noYawTurning_ and not this->useYawControl_){
					double yaw = atan2(this->goal_.pose.position.y - this->odom_.pose.pose.position.y, this->goal_.pose.position.x - this->odom_.pose.pose.position.x);
					this->facingYaw_ = yaw;
					this->moveToOrientation(yaw, this->desiredAngularVel_);
				}
				this->firstTimeSave_ = true;
				this->bsplineReplan_ = true;
				this->mpcReplan_ = true;
				this->refTrajReady_ = false;
				this->goalReceived_ = false;
				if (this->useGlobalPlanner_){
					cout << "[AutoFlight]: Start global planning." << endl;
					this->needGlobalPlan_ = true;
					this->globalPlanReady_ = false;
				}

				cout << "[AutoFlight]: Replan for new goal position." << endl; 
				return;
			}

			// return;
			if (this->bsplineTrajectoryReady_){
				if (this->bsplineHasCollision()){ // if trajectory not ready, do not replan
					this->bsplineReplan_ = true;
					this->mpcReplan_ = true;
					this->refTrajReady_ = false;
					cout << "[AutoFlight]: Replan for collision." << endl;
					return;
				}

				if (this->computeExecutionDistance() >= 1.5 and AutoFlight::getPoseDistance(this->odom_.pose.pose, this->goal_.pose) >= 3){
					this->bsplineReplan_ = true;
					this->mpcReplan_ = true;
					this->refTrajReady_ = false;
					cout << "[AutoFlight]: Regular replan." << endl;
					return;
				}
			}
			else{
				this->stop();
				this->refTrajReady_ = false;
				return;
			}

			if (this->mpcTrajectoryReady_){
				if (this->goalHasCollision()){
						this->mpcReplan_ = false;
						this->bsplineReplan_ = false;
						this->mpcTrajectoryReady_ = false;
						this->bsplineTrajectoryReady_ = false;
						ros::Rate r(200);
						while(ros::ok() and this->replanning_){
							r.sleep();
						}
						this->mpcTrajectoryReady_ = false;
						this->stop();
						this->refTrajReady_ = false;
						this->mpcFirstTime_ = true;
						cout<<"[AutoFlight]: Invalid goal. Stop!" << endl;
						return;
					}
				else if (this->mpcHasCollision() or this->hasDynamicCollision()){ 
					this->stop();
					this->mpcTrajectoryReady_ = false;
					this->mpcReplan_ = true;
					cout << "[AutoFlight]: Collision detected. MPC replan." << endl;
					return;
				}
				else if (AutoFlight::getPoseDistance(this->odom_.pose.pose, this->goal_.pose) <= 0.3){
					this->mpcReplan_ = false;
					this->mpcTrajectoryReady_ = false;
					ros::Rate r(200);
					while(ros::ok() and this->replanning_){
						r.sleep();
					}
					this->stop();
					this->refTrajReady_ = false;
					this->mpcTrajectoryReady_ = false;
					this->mpcFirstTime_ = true;
					cout << "[AutoFlight]: Goal reached. MPC Stop replan." << endl;
					return;
				}
			}
		}
	}

	void dynamicNavigation::trajExeCB(const ros::TimerEvent&){
		bool trajectoryReady;
		if (this->plannerType_ == PLANNER::MPC or this->plannerType_ == PLANNER::MIXED){
			trajectoryReady = this->mpcTrajectoryReady_;
		}
		else{
			trajectoryReady = this->bsplineTrajectoryReady_;
		}
		if (trajectoryReady){
			ros::Time currTime = ros::Time::now();
			double realTime = (currTime - this->trajStartTime_).toSec();
			Eigen::Vector3d pos, vel, acc;
			Eigen::Vector3d refPos;
			double endTime;
			if (this->plannerType_ == PLANNER::MPC){
				endTime = this->mpc_->getHorizon() * this->mpc_->getTs();
				pos = this->mpc_->getPos(realTime);
				vel = this->mpc_->getVel(realTime);
				acc = this->mpc_->getAcc(realTime);
				refPos = this->mpc_->getRef(realTime);
			}
			else if (this->plannerType_ == PLANNER::BSPLINE){
				this->trajTime_ = this->bsplineTraj_->getLinearReparamTime(realTime);
				double linearReparamFactor = this->bsplineTraj_->getLinearFactor();
				pos = this->trajectory_.at(this->trajTime_);
				vel = this->trajectory_.getDerivative().at(this->trajTime_) * linearReparamFactor;
				acc = this->trajectory_.getDerivative().getDerivative().at(this->trajTime_) * pow(linearReparamFactor, 2);
				endTime = this->trajectory_.getDuration()/linearReparamFactor;
			}
			else if (this->plannerType_ == PLANNER::MIXED){
				endTime = this->mpc_->getHorizon() * this->mpc_->getTs();
				pos = this->mpc_->getPos(realTime);
				vel = this->mpc_->getVel(realTime);
				acc = this->mpc_->getAcc(realTime);
				refPos = this->mpc_->getRef(realTime);
				this->trajTime_ = this->estimateExecutionTime();
			}

			double leftTime = endTime - realTime; 
			tracking_controller::Target target;
			if (leftTime <= 0.0){ // zero vel and zero acc if close to
				target.position.x = pos(0);
				target.position.y = pos(1);
				target.position.z = pos(2);
				target.velocity.x = 0.0;
				target.velocity.y = 0.0;
				target.velocity.z = 0.0;
				target.acceleration.x = 0.0;
				target.acceleration.y = 0.0;
				target.acceleration.z = 0.0;
				target.yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
				this->updateTargetWithState(target);						
			}
			else{
				if (not this->useYawControl_){
					target.yaw = this->facingYaw_;
				}
				else if (this->noYawTurning_){
					target.yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
				}
				else{
					if (this->plannerType_ == PLANNER::MPC or this->plannerType_ == PLANNER::MIXED){
						// smoothing yaw angle
						double forwardDist = 1.0;
						double dt = this->mpc_->getTs();
						bool noYawChange = true;
						for (double t=realTime; t<=endTime; t+=dt){
							// Eigen::Vector3d p = this->mpc_->getPos(t);
							Eigen::Vector3d p = this->mpc_->getRef(t);
							if ((p - refPos).norm() >= forwardDist){
								target.yaw = atan2(p(1) - refPos(1), p(0) - refPos(0));
								noYawChange = false;
								break;
							}
						}

						if (noYawChange){
							target.yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
						}
					}
					else{
						target.yaw = atan2(vel(1), vel(0));		
					}
				}				
				target.position.x = pos(0);
				target.position.y = pos(1);
				target.position.z = pos(2);
				target.velocity.x = vel(0);
				target.velocity.y = vel(1);
				target.velocity.z = vel(2);
				target.acceleration.x = acc(0);
				target.acceleration.y = acc(1);
				target.acceleration.z = acc(2);
				this->updateTargetWithState(target);						
			}
		}
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
		if (this->plannerType_ == PLANNER::MPC or this->plannerType_ == PLANNER::MIXED){
			if (this->mpcTrajMsg_.poses.size() != 0){
				this->mpcTrajPub_.publish(this->mpcTrajMsg_);
			}
		}
		if (this->plannerType_ == PLANNER::BSPLINE or this->plannerType_ == PLANNER::MIXED){
			if (this->bsplineTrajMsg_.poses.size() != 0){
				this->bsplineTrajPub_.publish(this->bsplineTrajMsg_);
			}
		}
		if (this->inputTrajMsg_.poses.size() != 0){
			this->inputTrajPub_.publish(this->inputTrajMsg_);
		}
		this->publishGoal();
	}

	void dynamicNavigation::freeMapCB(const ros::TimerEvent&){
		std::vector<onboardDetector::box3D> obstacles;
		std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> freeRegions;
		this->detector_->getObstacles(obstacles);
		double fov = 1.57;
		for (onboardDetector::box3D ob: obstacles){
			if (this->detector_->isObstacleInSensorRange(ob, fov)){
				Eigen::Vector3d lowerBound (ob.x-ob.x_width/2-0.3, ob.y-ob.y_width/2-0.3, ob.z-ob.z_width/2-0.3);
				Eigen::Vector3d upperBound (ob.x+ob.x_width/2+0.3, ob.y+ob.y_width/2+0.3, ob.z+ob.z_width/2+0.3);
				freeRegions.push_back(std::make_pair(lowerBound, upperBound));
			}
		}
		this->map_->updateFreeRegions(freeRegions);
		this->map_->freeRegions(freeRegions);
	}

	void dynamicNavigation::run(){
		// take off the drone
		this->takeoff();

		// register timer callback
		this->registerCallback();
	}

	void dynamicNavigation::getStartEndConditions(std::vector<Eigen::Vector3d>& startEndConditions){
		/*	
			1. start velocity
			2. start acceleration (set to zero)
			3. end velocity
			4. end acceleration (set to zero) 
		*/

		Eigen::Vector3d currVel = this->currVel_;
		Eigen::Vector3d currAcc = this->currAcc_;
		Eigen::Vector3d endVel (0.0, 0.0, 0.0);
		Eigen::Vector3d endAcc (0.0, 0.0, 0.0);

		// if (not this->bsplineTrajectoryReady_){
		// 	double yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		// 	Eigen::Vector3d direction (cos(yaw), sin(yaw), 0.0);
		// 	currVel = this->desiredVel_ * direction;
		// 	currAcc = this->desiredAcc_ * direction;
		// }
		startEndConditions.push_back(currVel);
		startEndConditions.push_back(endVel);
		startEndConditions.push_back(currAcc);
		startEndConditions.push_back(endAcc);
	}

	bool dynamicNavigation::goalHasCollision(){
		Eigen::Vector3d p;
		double r = 0.5;//radius for goal collision check
		for (double i=-r; i<=r;i+=0.1){
			for(double j=-r;j<=r;j+=0.1){
				for (double k = -r; k<=r; k+=0.1){
					p(0) = this->goal_.pose.position.x+i;
					p(1) = this->goal_.pose.position.y+j;
					p(2) = this->goal_.pose.position.z+k;
					if (this->map_->isInflatedOccupied(p)){
						return true;

					}
				}
			}
		}
		return false;
	}

	bool dynamicNavigation::mpcHasCollision(){
		if (this->mpcTrajectoryReady_){
			double dt = this->mpc_->getTs();
			ros::Time currTime = ros::Time::now();
			double startTime = std::min(1.0, (currTime-this->trajStartTime_).toSec());
			double endTime = std::min(startTime+2.0, this->mpc_->getHorizon()*dt);
			for (double t=startTime; t<=endTime; t+=dt){
				Eigen::Vector3d p = this->mpc_->getPos(t);
				bool hasCollision = this->map_->isInflatedOccupied(p);
				if (hasCollision){
					cout<<"[AutoFlight]: MPC collision detected!"<<endl;
					return true;
				}
			}
		}
		return false;
	}

	bool dynamicNavigation::bsplineHasCollision(){
		if (this->bsplineTrajectoryReady_){
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

	bool dynamicNavigation::hasCollision(){
		bool trajectoryReady;
		if (this->plannerType_ == PLANNER::MPC or this->plannerType_ == PLANNER::MIXED){
			trajectoryReady = this->mpcTrajectoryReady_;
		}
		else{
			trajectoryReady = this->bsplineTrajectoryReady_;
		}
		if (trajectoryReady){
			if (this->plannerType_ == PLANNER::MPC or this->plannerType_ == PLANNER::MIXED){
				double dt = this->mpc_->getTs();
				ros::Time currTime = ros::Time::now();
				double startTime = std::min(1.0, (currTime-this->trajStartTime_).toSec());
				double endTime = std::min(startTime+2.0, this->mpc_->getHorizon()*dt);
				for (double t=startTime; t<=endTime; t+=dt){
					Eigen::Vector3d p = this->mpc_->getPos(t);
					bool hasCollision = this->map_->isInflatedOccupied(p);
					if (hasCollision){
						cout<<"[AutoFlight]: MPC collision detected!"<<endl;
						return true;
					}
				}
			}
			else{
				for (double t=this->trajTime_; t<=this->trajectory_.getDuration(); t+=0.1){
					Eigen::Vector3d p = this->trajectory_.at(t);
					bool hasCollision = this->map_->isInflatedOccupied(p);
					if (hasCollision){
						return true;
					}
				}
			}
		}
		return false;
	}

	bool dynamicNavigation::hasDynamicCollision(){
		bool trajectoryReady;
		if (this->plannerType_ == PLANNER::MPC or this->plannerType_ == PLANNER::MIXED){
			trajectoryReady = this->mpcTrajectoryReady_;
		}
		else{
			trajectoryReady = this->bsplineTrajectoryReady_;
		}
		if (trajectoryReady){
			if (this->plannerType_ == PLANNER::MPC or this->plannerType_ == PLANNER::MIXED){
				std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
				if (this->useFakeDetector_){
					Eigen::Vector3d robotSize;
					this->map_->getRobotSize(robotSize);
					this->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize, robotSize);
				}
				else{ 
					this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
				}
				double dt = this->mpc_->getTs();
				ros::Time currTime = ros::Time::now();
				double startTime = std::min(1.0, (currTime-this->trajStartTime_).toSec());
				double endTime = std::min(startTime+1.0, this->mpc_->getHorizon()*dt);
				for (double t=startTime; t<=endTime; t+=dt){
					Eigen::Vector3d p = this->mpc_->getPos(t);
					for (size_t i=0; i<obstaclesPos.size(); ++i){
						Eigen::Vector3d ob = obstaclesPos[i];
						Eigen::Vector3d size = obstaclesSize[i];
						Eigen::Vector3d lowerBound = ob - size/2;
						Eigen::Vector3d upperBound = ob + size/2;
						if (p(0) >= lowerBound(0) and p(0) <= upperBound(0) and
							p(1) >= lowerBound(1) and p(1) <= upperBound(1) and
							p(2) >= lowerBound(2) and p(2) <= upperBound(2)){
							return true;
						}					
					}
				}
			}
			else if (this->plannerType_ == PLANNER::BSPLINE){
				std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
				if (this->useFakeDetector_){
					Eigen::Vector3d robotSize;
					this->map_->getRobotSize(robotSize);
					this->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize, robotSize);
				}
				else{ 
					this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
				}

				for (double t=this->trajTime_; t<=this->trajectory_.getDuration(); t+=0.1){
					Eigen::Vector3d p = this->trajectory_.at(t);
					
					for (size_t i=0; i<obstaclesPos.size(); ++i){
						Eigen::Vector3d ob = obstaclesPos[i];
						Eigen::Vector3d size = obstaclesSize[i];
						Eigen::Vector3d lowerBound = ob - size/2;
						Eigen::Vector3d upperBound = ob + size/2;
						if (p(0) >= lowerBound(0) and p(0) <= upperBound(0) and
							p(1) >= lowerBound(1) and p(1) <= upperBound(1) and
							p(2) >= lowerBound(2) and p(2) <= upperBound(2)){
							return true;
						}					
					}
				}
			}
		}
		return false;
	}

	double dynamicNavigation::estimateExecutionTime(){
		double dt = this->bsplineTraj_->getInitTs();
		double minDist = INFINITY;
		double time = -1;
		for (double t=0; t<=this->trajectory_.getDuration(); t+=dt){
			Eigen::Vector3d pos = this->trajectory_.at(t);
			if ((this->currPos_-pos).norm()<minDist){
				time = t;
				minDist = (this->currPos_-pos).norm();
			}
		}	
		return time;	
	}

	double dynamicNavigation::computeExecutionDistance(){
		if (this->bsplineTrajectoryReady_ and not this->bsplineReplan_){
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

	bool dynamicNavigation::replanForDynamicObstacle(){
		ros::Time currTime = ros::Time::now();
		std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
		if (this->useFakeDetector_){
			Eigen::Vector3d robotSize;
			this->map_->getRobotSize(robotSize);
			this->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize, robotSize);
		}
		else{ 
			this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
		}

		bool replan = false;
		bool hasDynamicObstacle = (obstaclesPos.size() != 0);
		if (hasDynamicObstacle){
			double timePassed = (currTime - this->lastDynamicObstacleTime_).toSec();
			if (this->lastDynamicObstacle_ == false or timePassed >= this->replanTimeForDynamicObstacle_){
				replan = true;
				this->lastDynamicObstacleTime_ = currTime;
			}
			this->lastDynamicObstacle_ = true;
		}
		else{
			this->lastDynamicObstacle_ = false;
		}

		return replan;
	}

	nav_msgs::Path dynamicNavigation::getCurrentTraj(double dt){
		nav_msgs::Path currentTraj;
		currentTraj.header.frame_id = "map";
		currentTraj.header.stamp = ros::Time::now();
	
		if (this->bsplineTrajectoryReady_){
			// include the current pose
			// geometry_msgs::PoseStamped psCurr;
			// psCurr.pose = this->odom_.pose.pose;
			// currentTraj.poses.push_back(psCurr);
			for (double t=this->trajTime_; t<=this->trajectory_.getDuration(); t+=dt){
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


	nav_msgs::Path dynamicNavigation::getRestGlobalPath(){
		nav_msgs::Path currPath;

		int nextIdx = this->rrtPathMsg_.poses.size()-1;
		Eigen::Vector3d pCurr (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		double minDist = std::numeric_limits<double>::infinity();
		for (size_t i=0; i<this->rrtPathMsg_.poses.size()-1; ++i){
			geometry_msgs::PoseStamped ps = this->rrtPathMsg_.poses[i];
			Eigen::Vector3d pEig (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
			Eigen::Vector3d pDiff = pCurr - pEig;

			geometry_msgs::PoseStamped psNext = this->rrtPathMsg_.poses[i+1];
			Eigen::Vector3d pEigNext (psNext.pose.position.x, psNext.pose.position.y, psNext.pose.position.z);
			Eigen::Vector3d diffToNext = pEigNext - pEig;
			double dist = (pEig - pCurr).norm();
			if (trajPlanner::angleBetweenVectors(diffToNext, pDiff) > PI_const*3.0/4.0){
				if (dist < minDist){
					nextIdx = i;
					minDist = dist;
				}
			}
		}


		geometry_msgs::PoseStamped psCurr;
		psCurr.pose = this->odom_.pose.pose;
		currPath.poses.push_back(psCurr);
		for (size_t i=nextIdx; i<this->rrtPathMsg_.poses.size(); ++i){
			currPath.poses.push_back(this->rrtPathMsg_.poses[i]);
		}
		return currPath;		
	}

	void dynamicNavigation::getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclesPos, std::vector<Eigen::Vector3d>& obstaclesVel, std::vector<Eigen::Vector3d>& obstaclesSize, const Eigen::Vector3d &robotSize){
		std::vector<onboardDetector::box3D> obstacles;
		this->detector_->getObstaclesInSensorRange(2*PI_const, obstacles, robotSize);
		for (onboardDetector::box3D ob : obstacles){
			Eigen::Vector3d pos (ob.x, ob.y, ob.z);
			Eigen::Vector3d vel (ob.Vx, ob.Vy, 0.0);
			Eigen::Vector3d size (ob.x_width, ob.y_width, ob.z_width);
			obstaclesPos.push_back(pos);
			obstaclesVel.push_back(vel);
			obstaclesSize.push_back(size);
		}
	}

	void dynamicNavigation::publishGoal(){
		nav_msgs::Path goal;
		if (this->usePredefinedGoal_){
			goal = this->predefinedGoal_;	
		}
		else{
			goal.poses = std::vector<geometry_msgs::PoseStamped> {this->goal_};
		}	
		if (goal.poses.size() != 0){
			visualization_msgs::MarkerArray msg;
			std::vector<visualization_msgs::Marker> pointVec;
			visualization_msgs::Marker point;
			int pointCount = 0;
			for (int i=0; i<int(goal.poses.size()); ++i){
				point.header.frame_id = "map";
				point.header.stamp = ros::Time::now();
				point.ns = "input_traj_points";
				point.id = pointCount;
				point.type = visualization_msgs::Marker::SPHERE;
				point.action = visualization_msgs::Marker::ADD;
				point.pose.position.x = goal.poses[i].pose.position.x;
				point.pose.position.y = goal.poses[i].pose.position.y;
				point.pose.position.z = goal.poses[i].pose.position.z;
				point.lifetime = ros::Duration(0.5);
				point.scale.x = 0.3;
				point.scale.y = 0.3;
				point.scale.z = 0.3;
				point.color.a = 1.0;
				point.color.r = 1;
				point.color.g = 0;
				point.color.b = 0;
				pointVec.push_back(point);
				++pointCount;			
			}
			msg.markers = pointVec;	
			this->goalPub_.publish(msg);
		}
	}
}