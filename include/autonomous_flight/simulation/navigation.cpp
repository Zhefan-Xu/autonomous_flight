/*
	FILE: navigation.cpp
	------------------------
	navigation implementation file in simulation
*/
#include <autonomous_flight/simulation/navigation.h>

namespace AutoFlight{
	navigation::navigation(const ros::NodeHandle& nh) : flightBase(nh){
		this->initParam();
		this->initModules();
		this->registerPub();
	}

	void navigation::initParam(){
    	// parameters    
		// Use global Planner
		if (not this->nh_.getParam("autonomous_flight/use_global_planner", this->useGlobalPlanner_)){
			this->useGlobalPlanner_ = false;
			cout << "[AutoFlight]: No use global planner param found. Use default: false." << endl;
		}
		else{
			cout << "[AutoFlight]: Use global planner is set to: " << this->useGlobalPlanner_ << "." << endl;
		}	

		//Use MPC Planner
		if (not this->nh_.getParam("autonomous_flight/use_mpc_planner", this->useMPCPlanner_)){
			this->useMPCPlanner_ = false;
			cout << "[AutoFlight]: No use MPC planner param found. Use default: false." << endl;
		}
		else{
			cout << "[AutoFlight]: Use MPC planner is set to: " << this->useMPCPlanner_ << "." << endl;
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

    	// trajectory data save path   	
		if (not this->nh_.getParam("autonomous_flight/trajectory_info_save_path", this->trajSavePath_)){
			this->trajSavePath_ = "No";
			cout << "[AutoFlight]: No trajectory info save path param found. Use current directory." << endl;
		}
		else{
			cout << "[AutoFlight]: Trajectory info save path is set to: " << this->trajSavePath_ << "." << endl;
		}	

		// whether or not to use time optimizer
		if (not this->nh_.getParam("autonomous_flight/use_time_optimizer", this->useTimeOptimizer_)){
			this->useTimeOptimizer_ = false;
			cout << "[AutoFlight]: No use time optimizer found. Use default: false." << endl;
		}
		else{
			cout << "[AutoFlight]: Use time optimizer is set to: " << this->useTimeOptimizer_ << "." << endl;
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
		this->polyTraj_->updateDesiredVel(this->desiredVel_);
		this->polyTraj_->updateDesiredAcc(this->desiredAcc_);

		// initialize piecewise linear trajectory planner
		this->pwlTraj_.reset(new trajPlanner::pwlTraj (this->nh_));

		if (this->useMPCPlanner_){
			this->mpc_.reset(new trajPlanner::mpcPlanner (this->nh_));
			this->mpc_->updateMaxVel(this->desiredVel_);
			this->mpc_->updateMaxAcc(this->desiredAcc_);
			this->mpc_->setMap(this->map_);
		}
		else{
			// initialize bspline trajectory planner
			this->bsplineTraj_.reset(new trajPlanner::bsplineTraj (this->nh_));
			this->bsplineTraj_->setMap(this->map_);
			this->bsplineTraj_->updateMaxVel(this->desiredVel_);
			this->bsplineTraj_->updateMaxAcc(this->desiredAcc_);

			// initialize the trajectory divider
			this->trajDivider_.reset(new timeOptimizer::trajDivider (this->nh_));
			this->trajDivider_->setMap(this->map_);

			// initialize the time optimizer
			this->timeOptimizer_.reset(new timeOptimizer::bsplineTimeOptimizer (this->nh_));
			this->timeOptimizer_->setMap(this->map_);
		}
	}

	void navigation::registerPub(){
		this->rrtPathPub_ = this->nh_.advertise<nav_msgs::Path>("navigation/rrt_path", 10);
		this->polyTrajPub_ = this->nh_.advertise<nav_msgs::Path>("navigation/poly_traj", 10);
		this->pwlTrajPub_ = this->nh_.advertise<nav_msgs::Path>("navigation/pwl_trajectory", 10);
		if (this->useMPCPlanner_){
			this->mpcTrajPub_ = this->nh_.advertise<nav_msgs::Path>("navigation/mpc_trajectory", 10);
		}
		else{
			this->bsplineTrajPub_ = this->nh_.advertise<nav_msgs::Path>("navigation/bspline_trajectory", 10);
		}
		this->inputTrajPub_ = this->nh_.advertise<nav_msgs::Path>("navigation/input_trajectory", 10);
		this->inputTrajPointsPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("navigation/input_trajetory_points", 10);
	}

	void navigation::registerCallback(){
		if (this->useMPCPlanner_){
			this->mpcTimer_ = this->nh_.createTimer(ros::Duration(0.1), &navigation::mpcCB, this);
		}
		else{
			// planner callback
			this->plannerTimer_ = this->nh_.createTimer(ros::Duration(0.1), &navigation::plannerCB, this);
		}	
		// collision check callback
		this->replanCheckTimer_ = this->nh_.createTimer(ros::Duration(0.01), &navigation::replanCheckCB, this);
		// trajectory execution callback
		this->trajExeTimer_ = this->nh_.createTimer(ros::Duration(0.05), &navigation::trajExeCB, this);
		// this->trajExeWorker_ = std::thread(&navigation::trajExeCB, this);
		// this->trajExeWorker_.detach();
		// visualization callback
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.033), &navigation::visCB, this);
	}

	void navigation::mpcCB(const ros::TimerEvent&){
		// ros::Time mpcCBStartTime = ros::Time::now();
		// cout<<"[AutoFlight]: mpc Callback time: "<<(mpcCBEndTime-mpcCBStartTime).toSec()<<endl;
		if (this->replan_){
			if (not this->refTrajReady_){
				if (this->useGlobalPlanner_){
					this->rrtPlanner_->updateStart(this->odom_.pose.pose);
					this->rrtPlanner_->updateGoal(this->goal_.pose);
					nav_msgs::Path rrtPathMsgTemp;
					this->rrtPlanner_->makePlan(rrtPathMsgTemp);
					if (rrtPathMsgTemp.poses.size() >= 2){
						this->rrtPathMsg_ = rrtPathMsgTemp;
						// this->globalPlanReady_ = true;
						// this->refTrajReady_ = false;
					}
					// this->needGlobalPlan_ = false;
					Eigen::Vector3d startVel (0, 0, 0);
					Eigen::Vector3d startAcc (0, 0, 0);
					Eigen::Vector3d endVel (0, 0, 0);
					Eigen::Vector3d endAcc (0, 0, 0);
					std::vector<Eigen::Vector3d> startEndConditions {startVel, startAcc, endVel, endAcc};

					this->polyTraj_->updatePath(rrtPathMsgTemp, startEndConditions);
					this->polyTraj_->makePlan(this->polyTrajMsg_); // no corridor constraint		
					//TODO: Collision check or not
					double dt = 0.1;//TODO::add in param?
					nav_msgs::Path mpcInputTraj = this->polyTraj_->getTrajectory(dt);
					this->mpc_->updatePath(mpcInputTraj,dt);
					this->inputTrajMsg_ = mpcInputTraj;
					//update real goal
					// this->goal_ = mpcInputTraj
					this->refTrajReady_ = true;
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
					this->polyTraj_->makePlan(this->polyTrajMsg_); // no corridor constraint

					double dt = 0.1;//TODO::add in param?
					nav_msgs::Path mpcInputTraj = this->polyTraj_->getTrajectory(dt);
					this->mpc_->updatePath(mpcInputTraj,dt);
					this->inputTrajMsg_ = mpcInputTraj;
					this->refTrajReady_ = true;
				}
			}
			else if (this->refTrajReady_){
				Eigen::Vector3d currPos, currVel;
				this->getCurrentStates(currPos, currVel);
				this->mpc_->updateCurrStates(currPos, currVel);
				ros::Time trajStartTime = ros::Time::now();
				// cout<<"mpc freq: "<<(trajStartTime-this->trajStartTime_).toSec()<<endl;
				bool newTrajReturn = mpc_->makePlanCG();
				nav_msgs::Path mpcTraj;
				this->mpc_->getTrajectory(mpcTraj);
				

				if (this->hasCollision()){
					this->stop();
					this->trajectoryReady_ = false;
					// this->refTrajReady_ = false;
					// this->replan_ = false;
				}
				else{	
					if (this->mpcFirstTime_ and not newTrajReturn){
						this->trajectoryReady_ = false;
					}
					else{
						this->mpcTrajMsg_ = mpcTraj;			
						if (newTrajReturn){
							this->trajStartTime_ = trajStartTime;
						}
						this->trajectoryReady_ = true;
					}
				}
			}
		}
		// ros::Time mpcCBEndTime = ros::Time::now();
		// cout<<"[AutoFlight]: mpc Callback time: "<<(mpcCBEndTime-mpcCBStartTime).toSec()<<endl;
	}

	void navigation::getCurrentStates(Eigen::Vector3d &currPos, Eigen::Vector3d &currVel){
		currPos <<this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z;
		currVel = this->currVel_;
	}

	void navigation::plannerCB(const ros::TimerEvent&){
		if (not this->firstGoal_) return;

		if (this->replan_){
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
						// startEndConditions[3] = Eigen::Vector3d (0.0, 0.0, 0.0);	
						// startEndConditions[3] = this->polyTraj_->getAcc(finalTime);	

					}
					else{
						cout << "[AutoFlight]: Global planner fails. Check goal and map." << endl;
					}		
				}				
			}
			else{
				if (not this->trajectoryReady_){ // use polynomial trajectory as input
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
					// this->bsplineTraj_->updateControlPointsTs(dtTemp);
					// cout << "time step to sample is: " << dtTemp << endl;
					// startEndConditions[3] = Eigen::Vector3d (0.0, 0.0, 0.0);
					// startEndConditions[3] = this->polyTraj_->getAcc(finalTime);
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
						// startEndConditions[3] = Eigen::Vector3d (0.0, 0.0, 0.0);
						// startEndConditions[3] = this->polyTraj_->getAcc(finalTime);
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
			

			this->inputTrajMsg_ = inputTraj;

			bool updateSuccess = this->bsplineTraj_->updatePath(inputTraj, startEndConditions);
			if (updateSuccess){
				nav_msgs::Path bsplineTrajMsgTemp;
				bool planSuccess = this->bsplineTraj_->makePlan(bsplineTrajMsgTemp);
				if (planSuccess){
					this->bsplineTrajMsg_ = bsplineTrajMsgTemp;
					this->trajStartTime_ = ros::Time::now();
					this->trajTime_ = 0.0; // reset trajectory time
					this->trajectory_ = this->bsplineTraj_->getTrajectory();

					// optimize time
					if (this->useTimeOptimizer_){
						ros::Time timeOptStartTime = ros::Time::now();
						this->timeOptimizer_->optimize(this->trajectory_, this->desiredVel_, this->desiredAcc_, 0.1);
						ros::Time timeOptEndTime = ros::Time::now();
						cout << "[AutoFlight]: Time optimizatoin spends: " << (timeOptEndTime - timeOptStartTime).toSec() << "s." << endl;
					}
					this->trajectoryReady_ = true;
					this->replan_ = false;
					cout << "\033[1;32m[AutoFlight]: Trajectory generated successfully.\033[0m " << endl;

					if (this->trajSavePath_ != "No" and this->firstTimeSave_){
						this->bsplineTraj_->writeCurrentTrajInfo(this->trajSavePath_, 0.05);
						this->firstTimeSave_ = false;
					}
				}
				else{
					// if the current trajectory is still valid, then just ignore this iteration
					// if the current trajectory/or new goal point is assigned is not valid, then just stop
					if (this->hasCollision()){
						this->trajectoryReady_ = false;
						this->stop();
						cout << "[AutoFlight]: Stop!!! Trajectory generation fails." << endl;
						this->replan_ = false;
					}
					else{
						if (this->trajectoryReady_){
							cout << "[AutoFlight]: Trajectory fail. Use trajectory from previous iteration." << endl;
							this->replan_ = false;
						}
						else{
							cout << "[AutoFlight]: Unable to generate a feasible trajectory. Please provide a new goal." << endl;
							this->replan_ = false;
						}
					}
				}
			}
			else{
				this->trajectoryReady_ = false;
				this->stop();
				this->replan_ = false;
				cout << "[AutoFlight]: Goal is not valid. Stop." << endl;
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
		if(this->useMPCPlanner_){
			if (this->goalReceived_){
				if(this->goalHasCollision()){
					this->replan_ = false;
					this->refTrajReady_ = false;
					this->goalReceived_ = false;
					cout << "[AutoFlight]: Invalid goal position, please assign a new goal" << endl; 
					return;
				}
				else{
					this->replan_ = false;
					this->refTrajReady_ = false;
					if (not this->noYawTurning_ and not this->useYawControl_){
						double yaw = atan2(this->goal_.pose.position.y - this->odom_.pose.pose.position.y, this->goal_.pose.position.x - this->odom_.pose.pose.position.x);
						this->facingYaw_ = yaw;
						this->moveToOrientation(yaw, this->desiredAngularVel_);
					}
					this->firstTimeSave_ = true;
					this->replan_ = true;
					this->goalReceived_ = false;
					if (this->useGlobalPlanner_){
						cout << "[AutoFlight]: Start global planning." << endl;
						// this->needGlobalPlan_ = true;
						// this->globalPlanReady_ = false;
					}

					cout << "[AutoFlight]: Replan for new goal position." << endl; 
					return;
				}
			}
			
			if (this->trajectoryReady_){
				if (this->hasCollision()){ 
					this->stop();
					this->trajectoryReady_ = false;
					this->replan_ = true;
					cout << "[AutoFlight]: Collision detected, replan." << endl;
					return;
				}
				else if(AutoFlight::getPoseDistance(this->odom_.pose.pose, this->goal_.pose) <= 0.3){
					this->stop();
					this->replan_ = false;
					this->refTrajReady_ = false;
					this->trajectoryReady_ = false;
					this->mpcFirstTime_ = true;
					cout << "[AutoFlight]: Goal reached, stop replan." << endl;
					return;
				}
			}
		}
		else{
			if (this->goalReceived_){
				this->replan_ = false;
				this->trajectoryReady_ = false;
				if (not this->noYawTurning_ and not this->useYawControl_){
					double yaw = atan2(this->goal_.pose.position.y - this->odom_.pose.pose.position.y, this->goal_.pose.position.x - this->odom_.pose.pose.position.x);
					this->facingYaw_ = yaw;
					this->moveToOrientation(yaw, this->desiredAngularVel_);
				}
				this->firstTimeSave_ = true;
				this->replan_ = true;
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
			if (this->trajectoryReady_){
				if (this->hasCollision()){ // if trajectory not ready, do not replan
					this->replan_ = true;
					cout << "[AutoFlight]: Replan for collision." << endl;
					return;
				}

				if (this->computeExecutionDistance() >= 1.5 and AutoFlight::getPoseDistance(this->odom_.pose.pose, this->goal_.pose) >= 3){
					this->replan_ = true;
					cout << "[AutoFlight]: Regular replan." << endl;
					return;
				}
			}
		}
		return;
	}

	void navigation::trajExeCB(const ros::TimerEvent&){
		// ros::Rate r(20);
		// while(ros::ok()){
		if (this->trajectoryReady_){
			ros::Time currTime = ros::Time::now();
			double realTime = (currTime - this->trajStartTime_).toSec();
			Eigen::Vector3d pos, vel, acc;
			double endTime;
			if (this->useMPCPlanner_){
				pos = this->mpc_->getPos(realTime);
				vel = this->mpc_->getVel(realTime);
				acc = this->mpc_->getAcc(realTime);
				endTime = this->mpc_->getHorizon()*this->mpc_->getTs();
			}
			else{
				if (this->useTimeOptimizer_){
					this->trajTime_ = this->timeOptimizer_->getStates(realTime, pos, vel, acc);
					endTime = this->timeOptimizer_->getDuration();
				}
				else{
					this->trajTime_ = this->bsplineTraj_->getLinearReparamTime(realTime);
					double linearReparamFactor = this->bsplineTraj_->getLinearFactor();
					pos = this->trajectory_.at(this->trajTime_);
					vel = this->trajectory_.getDerivative().at(this->trajTime_) * linearReparamFactor;
					acc = this->trajectory_.getDerivative().getDerivative().at(this->trajTime_) * pow(linearReparamFactor, 2);
					endTime = this->trajectory_.getDuration()/linearReparamFactor;
				}
			}


			double leftTime = endTime - realTime; 
			// cout << "left time: " << leftTime << endl;
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
					double targetYaw = atan2(vel(1), vel(0));
					double currentYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
					if (targetYaw>currentYaw){
						target.yaw = std::max(targetYaw,currentYaw+PI_const/1800);
					}
					else{
						target.yaw = std::min(targetYaw,currentYaw-PI_const/1800);
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
		// // ros::spinOnce();
		// r.sleep();
		// }
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
		if (this->useMPCPlanner_){
			if (this->mpcTrajMsg_.poses.size() != 0){
				this->mpcTrajPub_.publish(this->mpcTrajMsg_);
			}
		}
		else{
			if (this->bsplineTrajMsg_.poses.size() != 0){
				this->bsplineTrajPub_.publish(this->bsplineTrajMsg_);
			}
		}

		this->publishInputTraj();
	}

	void navigation::run(){
		// take off the drone
		this->takeoff();
		// int temp1 = system("mkdir ~/rosbag_navigation_info &");
		// int temp2 = system("mv ~/rosbag_navigation_info/exploration_info.bag ~/rosbag_navigation_info/previous.bag &");
		// int temp3 = system("rosbag record -O ~/rosbag_navigation_info/navigation_info.bag /camera/color/image_raw /occupancy_map/inflated_voxel_map /navigation/bspline_trajectory /mavros/local_position/pose /mavros/setpoint_position/local /tracking_controller/vel_and_acc_info /tracking_controller/target_pose /tracking_controller/trajectory_history /trajDivider/braking_zone /trajDivider/kdtree_range __name:=navigation_bag_info &");
		// if (temp1==-1 or temp2==-1 or temp3==-1){
		// 	cout << "[AutoFlight]: Recording fails." << endl;
		// }
		// register timer callback
		this->registerCallback();
	}


	void navigation::getStartEndConditions(std::vector<Eigen::Vector3d>& startEndConditions){
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

		// if (not this->trajectoryReady_){
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

	bool navigation::goalHasCollision(){
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

	bool navigation::hasCollision(){
		if (this->trajectoryReady_){
			if (this->useMPCPlanner_){
				double dt = this->mpc_->getTs();
				ros::Time currTime = ros::Time::now();
				double startTime = (currTime-this->trajStartTime_).toSec();
				double endTime = std::min(startTime+2.0, this->mpc_->getHorizon()*dt);
				// cout<<"collision check: start time  "<<startTime<<"end time: "<< endTime<<endl;
				for (double t = startTime; t<=endTime; t+=dt){
					Eigen::Vector3d p = this->mpc_->getPos(t);
					bool hasCollision = this->map_->isInflatedOccupied(p);
					if (hasCollision){
						cout<<"[AutoFlight]: Collision detected !"<<endl;
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

	double navigation::computeExecutionDistance(){
		if (this->trajectoryReady_ and not this->replan_){
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

	nav_msgs::Path navigation::getRestGlobalPath(){
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


	void navigation::publishInputTraj(){
		// this function publishes the input path as trajectory and also sample points
		if (this->inputTrajMsg_.poses.size() != 0){
			// publish the input trajectory as a smooth path
			this->inputTrajPub_.publish(this->inputTrajMsg_);

			visualization_msgs::MarkerArray msg;
			std::vector<visualization_msgs::Marker> pointVec;
			visualization_msgs::Marker point;
			int pointCount = 0;
			for (int i=0; i<int(this->inputTrajMsg_.poses.size()); ++i){
				point.header.frame_id = "map";
				point.header.stamp = ros::Time::now();
				point.ns = "input_traj_points";
				point.id = pointCount;
				point.type = visualization_msgs::Marker::SPHERE;
				point.action = visualization_msgs::Marker::ADD;
				point.pose.position.x = this->inputTrajMsg_.poses[i].pose.position.x;
				point.pose.position.y = this->inputTrajMsg_.poses[i].pose.position.y;
				point.pose.position.z = this->inputTrajMsg_.poses[i].pose.position.z;
				point.lifetime = ros::Duration(0.1);
				point.scale.x = 0.2;
				point.scale.y = 0.2;
				point.scale.z = 0.2;
				point.color.a = 1.0;
				point.color.r = 0;
				point.color.g = 1;
				point.color.b = 0;
				pointVec.push_back(point);
				++pointCount;			
			}
			msg.markers = pointVec;	
			this->inputTrajPointsPub_.publish(msg);
		}
	}
}
