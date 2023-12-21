/*
	FILE: dynamicExploration.cpp
	-----------------------------
	Implementation of dynamic exploration
*/

#include <autonomous_flight/px4/dynamicExploration.h>

namespace AutoFlight{
	dynamicExploration::dynamicExploration(const ros::NodeHandle& nh) : flightBase(nh){
		this->initParam();
		this->initModules();
		this->registerPub();
		if (this->useFakeDetector_){
			// free map callback
			this->freeMapTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dynamicExploration::freeMapCB, this);
		}
	}

	void dynamicExploration::initParam(){
    	// use simulation detector	
		if (not this->nh_.getParam("autonomous_flight/use_fake_detector", this->useFakeDetector_)){
			this->useFakeDetector_ = false;
			cout << "[AutoFlight]: No use fake detector param found. Use default: false." << endl;
		}
		else{
			cout << "[AutoFlight]: Use fake detector is set to: " << this->useFakeDetector_ << "." << endl;
		}
		
		// desired velocity
		if (not this->nh_.getParam("autonomous_flight/desired_velocity", this->desiredVel_)){
			this->desiredVel_ = 0.5;
			cout << "[AutoFlight]: No desired velocity param. Use default 0.5m/s." << endl;
		}
		else{
			cout << "[AutoFlight]: Desired velocity is set to: " << this->desiredVel_ << "m/s." << endl;
		}	

		// desired acceleration
		if (not this->nh_.getParam("autonomous_flight/desired_acceleration", this->desiredAcc_)){
			this->desiredAcc_ = 2.0;
			cout << "[AutoFlight]: No desired acceleration param. Use default 2.0m/s^2." << endl;
		}
		else{
			cout << "[AutoFlight]: Desired acceleration is set to: " << this->desiredAcc_ << "m/s^2." << endl;
		}	

		//  desired angular velocity
		if (not this->nh_.getParam("autonomous_flight/desired_angular_velocity", this->desiredAngularVel_)){
			this->desiredAngularVel_ = 0.5;
			cout << "[AutoFlight]: No angular velocity param. Use default 0.5rad/s." << endl;
		}
		else{
			cout << "[AutoFlight]: Angular velocity is set to: " << this->desiredAngularVel_ << "rad/s." << endl;
		}	

		//  desired angular velocity
		if (not this->nh_.getParam("autonomous_flight/waypoint_stablize_time", this->wpStablizeTime_)){
			this->wpStablizeTime_ = 1.0;
			cout << "[AutoFlight]: No waypoint stablize time param. Use default 1.0s." << endl;
		}
		else{
			cout << "[AutoFlight]: Waypoint stablize time is set to: " << this->wpStablizeTime_ << "s." << endl;
		}	

		//  initial scan
		if (not this->nh_.getParam("autonomous_flight/initial_scan", this->initialScan_)){
			this->initialScan_ = false;
			cout << "[AutoFlight]: No initial scan param. Use default 1.0s." << endl;
		}
		else{
			cout << "[AutoFlight]: Initial scan is set to: " << this->initialScan_ << endl;
		}	

    	// replan time for dynamic obstacle
		if (not this->nh_.getParam("autonomous_flight/replan_time_for_dynamic_obstacles", this->replanTimeForDynamicObstacle_)){
			this->replanTimeForDynamicObstacle_ = 0.3;
			cout << "[AutoFlight]: No dynamic obstacle replan time param found. Use default: 0.3s." << endl;
		}
		else{
			cout << "[AutoFlight]: Dynamic obstacle replan time is set to: " << this->replanTimeForDynamicObstacle_ << "s." << endl;
		}	

    	// free range 
    	std::vector<double> freeRangeTemp;
		if (not this->nh_.getParam("autonomous_flight/free_range", freeRangeTemp)){
			this->freeRange_ = Eigen::Vector3d (2, 2, 1);
			cout << "[AutoFlight]: No free range param found. Use default: [2, 2, 1]m." << endl;
		}
		else{
			this->freeRange_(0) = freeRangeTemp[0];
			this->freeRange_(1) = freeRangeTemp[1];
			this->freeRange_(2) = freeRangeTemp[2];
			cout << "[AutoFlight]: Free range is set to: " << this->freeRange_.transpose() << "m." << endl;
		}	

    	// reach goal distance
		if (not this->nh_.getParam("autonomous_flight/reach_goal_distance", this->reachGoalDistance_)){
			this->reachGoalDistance_ = 0.1;
			cout << "[AutoFlight]: No reach goal distance param found. Use default: 0.1m." << endl;
		}
		else{
			cout << "[AutoFlight]: Reach goal distance is set to: " << this->reachGoalDistance_ << "m." << endl;
		}	
	}

	void dynamicExploration::initModules(){
		// initialize map
		if (this->useFakeDetector_){
			// initialize fake detector
			this->detector_.reset(new onboardDetector::fakeDetector (this->nh_));	
			this->map_.reset(new mapManager::dynamicMap (this->nh_, false));
		}
		else{
			this->map_.reset(new mapManager::dynamicMap (this->nh_));
		}

		// initialize exploration planner
		this->expPlanner_.reset(new globalPlanner::DEP (this->nh_));
		this->expPlanner_->setMap(this->map_);
		this->expPlanner_->loadVelocity(this->desiredVel_, this->desiredAngularVel_);

		// initialize polynomial trajectory planner
		this->polyTraj_.reset(new trajPlanner::polyTrajOccMap (this->nh_));
		this->polyTraj_->setMap(this->map_);
		this->polyTraj_->updateDesiredVel(this->desiredVel_);
		this->polyTraj_->updateDesiredAcc(this->desiredAcc_);

		// initialize piecewise linear trajectory planner
		this->pwlTraj_.reset(new trajPlanner::pwlTraj (this->nh_));

		// initialize bspline trajectory planner
		this->bsplineTraj_.reset(new trajPlanner::bsplineTraj (this->nh_));
		this->bsplineTraj_->setMap(this->map_);
		this->bsplineTraj_->updateMaxVel(this->desiredVel_);
		this->bsplineTraj_->updateMaxAcc(this->desiredAcc_);		
	}

	void dynamicExploration::registerCallback(){
		// initialize exploration planner replan in another thread
		this->exploreReplanWorker_ = std::thread(&dynamicExploration::exploreReplan, this);
		this->exploreReplanWorker_.detach();

		// exploration callback
		// this->explorationTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dynamicExploration::explorationCB, this);

		// planner callback
		this->plannerTimer_ = this->nh_.createTimer(ros::Duration(0.02), &dynamicExploration::plannerCB, this);

		// replan check timer
		this->replanCheckTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dynamicExploration::replanCheckCB, this);
		
		// trajectory execution callback
		this->trajExeTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dynamicExploration::trajExeCB, this);
	
		// visualization execution callabck
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.033), &dynamicExploration::visCB, this);
	}

	void dynamicExploration::registerPub(){
		this->polyTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicExploration/poly_traj", 1000);
		this->pwlTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicExploration/pwl_trajectory", 1000);
		this->bsplineTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicExploration/bspline_trajectory", 1000);
		this->inputTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicExploration/input_trajectory", 1000);
	}

	void dynamicExploration::explorationCB(const ros::TimerEvent&){
		if (this->explorationReplan_){
			this->expPlanner_->setMap(this->map_);
			ros::Time startTime = ros::Time::now();
			bool replanSuccess = this->expPlanner_->makePlan();
			if (replanSuccess){
				this->waypoints_ = this->expPlanner_->getBestPath();
				this->newWaypoints_ = true;
				this->waypointIdx_ = 1;
				this->explorationReplan_ = false;
			}
			ros::Time endTime = ros::Time::now();
			cout << "[AutoFlight]: DEP planning time: " << (endTime - startTime).toSec() << "s." << endl;			
		}
	}

	void dynamicExploration::plannerCB(const ros::TimerEvent&){
		// cout << "in planner callback" << endl;

		if (this->replan_){
			std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
			if (this->useFakeDetector_){
				this->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}
			else{ 
				this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}			
			nav_msgs::Path inputTraj;
			std::vector<Eigen::Vector3d> startEndConditions;
			this->getStartEndConditions(startEndConditions); 
			// double initTs = this->bsplineTraj_->getInitTs();

			// generate new trajectory
			nav_msgs::Path simplePath;
			geometry_msgs::PoseStamped pStart, pGoal;
			pStart.pose = this->odom_.pose.pose;
			pGoal = this->goal_;
			simplePath.poses = {pStart, pGoal};
			this->pwlTraj_->updatePath(simplePath, false);
			this->pwlTraj_->makePlan(inputTraj, this->bsplineTraj_->getControlPointDist());
			// if (not this->trajectoryReady_){
			// 	// generate new trajectory
			// 	nav_msgs::Path simplePath;
			// 	geometry_msgs::PoseStamped pStart, pGoal;
			// 	pStart.pose = this->odom_.pose.pose;
			// 	pGoal = this->goal_;
			// 	simplePath.poses = {pStart, pGoal};
			// 	this->pwlTraj_->updatePath(simplePath, false);
			// 	this->pwlTraj_->makePlan(inputTraj, this->bsplineTraj_->getControlPointDist());
			// }
			// else{
			// 	Eigen::Vector3d bsplineLastPos = this->trajectory_.at(this->trajectory_.getDuration());
			// 	geometry_msgs::PoseStamped lastPs; lastPs.pose.position.x = bsplineLastPos(0); lastPs.pose.position.y = bsplineLastPos(1); lastPs.pose.position.z = bsplineLastPos(2);
			// 	Eigen::Vector3d goalPos (this->goal_.pose.position.x, this->goal_.pose.position.y, this->goal_.pose.position.z);
			// 	// if ((bsplineLastPos - goalPos).norm() >= 0.1){
			// 	nav_msgs::Path inputPWLTraj;
			// 	nav_msgs::Path simplePath;
			// 	geometry_msgs::PoseStamped pStart, pGoal;
			// 	pStart = lastPs;
			// 	pGoal = this->goal_;
			// 	simplePath.poses = {pStart, pGoal};
			// 	this->pwlTraj_->updatePath(simplePath, false);
			// 	this->pwlTraj_->makePlan(inputPWLTraj, this->bsplineTraj_->getControlPointDist());


			// 	nav_msgs::Path adjustedInputCombinedTraj;
			// 	bool satisfyDistanceCheck = false;
			// 	double dtTemp = initTs;
			// 	double finalTimeTemp;
			// 	ros::Time startTime = ros::Time::now();
			// 	ros::Time currTime;
			// 	while (ros::ok()){
			// 		currTime = ros::Time::now();
			// 		if ((currTime - startTime).toSec() >= 0.05){
			// 			cout << "[AutoFlight]: Exceed path check time. Use the best." << endl;
			// 			break;
			// 		}							
			// 		nav_msgs::Path inputRestTraj = this->getCurrentTraj(dtTemp);
			// 		nav_msgs::Path inputCombinedTraj;
			// 		inputCombinedTraj.poses = inputRestTraj.poses;
			// 		for (size_t i=1; i<inputPWLTraj.poses.size(); ++i){
			// 			inputCombinedTraj.poses.push_back(inputPWLTraj.poses[i]);
			// 		}
					
			// 		satisfyDistanceCheck = this->bsplineTraj_->inputPathCheck(inputCombinedTraj, adjustedInputCombinedTraj, dtTemp, finalTimeTemp);
			// 		if (satisfyDistanceCheck) break;
					
			// 		dtTemp *= 0.8; // magic number 0.8
			// 	}
			// 	inputTraj = adjustedInputCombinedTraj;
			// 	// }
			// 	// else{
			// 	// 	nav_msgs::Path adjustedInputRestTraj;
			// 	// 	bool satisfyDistanceCheck = false;
			// 	// 	double dtTemp = initTs;
			// 	// 	double finalTimeTemp;
			// 	// 	ros::Time startTime = ros::Time::now();
			// 	// 	ros::Time currTime;
			// 	// 	while (ros::ok()){
			// 	// 		currTime = ros::Time::now();
			// 	// 		if ((currTime - startTime).toSec() >= 0.05){
			// 	// 			cout << "[AutoFlight]: Exceed path check time. Use the best." << endl;
			// 	// 			break;
			// 	// 		}
			// 	// 		nav_msgs::Path inputRestTraj = this->getCurrentTraj(dtTemp);
			// 	// 		satisfyDistanceCheck = this->bsplineTraj_->inputPathCheck(inputRestTraj, adjustedInputRestTraj, dtTemp, finalTimeTemp);
			// 	// 		if (satisfyDistanceCheck) break;
						
			// 	// 		dtTemp *= 0.8;
			// 	// 	}
			// 	// 	inputTraj = adjustedInputRestTraj;					
			// 	// }

			// }

			

			this->inputTrajMsg_ = inputTraj;
			bool updateSuccess = this->bsplineTraj_->updatePath(inputTraj, startEndConditions);
			if (obstaclesPos.size() != 0 and updateSuccess){
				this->bsplineTraj_->updateDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}
			if (updateSuccess){
				nav_msgs::Path bsplineTrajMsgTemp;
				bool planSuccess = this->bsplineTraj_->makePlan(bsplineTrajMsgTemp);
				if (planSuccess){
					this->bsplineTrajMsg_ = bsplineTrajMsgTemp;
					this->trajStartTime_ = ros::Time::now();
					this->trajTime_ = 0.0; // reset trajectory time
					this->trajectory_ = this->bsplineTraj_->getTrajectory();

					// optimize time
					// ros::Time timeOptStartTime = ros::Time::now();
					// this->timeOptimizer_->optimize(this->trajectory_, this->desiredVel_, this->desiredAcc_, 0.1);
					// ros::Time timeOptEndTime = ros::Time::now();
					// cout << "[AutoFlight]: Time optimizatoin spends: " << (timeOptEndTime - timeOptStartTime).toSec() << "s." << endl;

					this->trajectoryReady_ = true;
					this->replan_ = false;
					cout << "\033[1;32m[AutoFlight]: Trajectory generated successfully.\033[0m " << endl;
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
					else if (this->hasDynamicCollision()){
						this->trajectoryReady_ = false;
						this->stop();
						cout << "[AutoFlight]: Stop!!! Trajectory generation fails. Replan for dynamic obstacles." << endl;
						this->replan_ = true;
					}
					else{
						if (this->trajectoryReady_){
							cout << "[AutoFlight]: Trajectory fail. Use trajectory from previous iteration." << endl;
							this->replan_ = false;
						}
						else{
							cout << "[AutoFlight]: Unable to generate a feasible trajectory." << endl;
							cout << "\033[1;32m[AutoFlight]: Wait for new path. Press ENTER to Replan.\033[0m" << endl;
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

	void dynamicExploration::replanCheckCB(const ros::TimerEvent&){
		/*
			Replan if
			1. collision detected
			2. new goal point assigned
			3. fixed distance
		*/

		if (this->newWaypoints_){
			this->replan_ = false;
			this->trajectoryReady_ = false;
			double yaw = atan2(this->waypoints_.poses[1].pose.position.y - this->odom_.pose.pose.position.y, this->waypoints_.poses[1].pose.position.x - this->odom_.pose.pose.position.x);
			// cout << "[AutoFlight]: Go to next waypoint. Press ENTER to continue rotation." << endl;
			// std::cin.clear();
			// fflush(stdin);
			// std::cin.get();
			this->moveToOrientation(yaw, this->desiredAngularVel_);
			// cout << "[AutoFlight]: Press ENTER to move forward." << endl;
			// std::cin.clear();
			// fflush(stdin);
			// std::cin.get();		
			this->replan_ = true;
			this->newWaypoints_ = false;
			if (this->waypointIdx_ < int(this->waypoints_.poses.size())){
				this->goal_ = this->waypoints_.poses[this->waypointIdx_];
			}
			++this->waypointIdx_;
			cout << "[AutoFlight]: Replan for new waypoints." << endl; 

			return;
		}

		// if (this->isReach(this->goal_, 0.1, false) and this->waypointIdx_ <= int(this->waypoints_.poses.size())){
		// cout << "outside the if" << endl;
		// cout << "waypoints size: " << this->waypoints_.poses.size() << endl;
		// cout << "current waypoint idx: " << this->waypointIdx_ << endl;
		if (this->waypoints_.poses.size() != 0 and this->isReach(this->goal_, this->reachGoalDistance_, false) and this->waypointIdx_ <= int(this->waypoints_.poses.size())){
			// cout << "1" << endl;
			// when reach current goal point, reset replan and trajectory ready
			this->replan_ = false;
			this->trajectoryReady_ = false;
			// cout << "[AutoFlight]: Go to next waypoint. Press ENTER to continue rotation." << endl;
			// std::cin.clear();
			// fflush(stdin);
			// std::cin.get();
			cout << "[AutoFlight]: Rotate and replan..." << endl;
			geometry_msgs::Quaternion quat = this->goal_.pose.orientation;
			double yaw = AutoFlight::rpy_from_quaternion(quat);
			this->waitTime(this->wpStablizeTime_);
			this->moveToOrientation(yaw, this->desiredAngularVel_);
			cout << "[AutoFlight]: Finish rotation." << endl;
			// cout << "[AutoFlight]: Press ENTER to move forward." << endl;
			// std::cin.clear();
			// fflush(stdin);
			// std::cin.get();	

			// change current goal
			if (this->waypointIdx_ < int(this->waypoints_.poses.size())){
				this->goal_ = this->waypoints_.poses[this->waypointIdx_];
			}
			if (this->waypointIdx_ + 1 > int(this->waypoints_.poses.size())){
				cout << "\033[1;32m[AutoFlight]: Finishing entire path. Wait for new path. Press ENTER to Replan.\033[0m" << endl;
				this->replan_ = false;
				// this->explorationReplan_ = true;
			}
			else{
				cout << "[AutoFlight]: Start planning for next waypoint." << endl;
				this->replan_ = true;
			}
			++this->waypointIdx_;
			this->trajectoryReady_ = false;
	
			return;		
		}
		else if (this->waypoints_.poses.size() != 0 and this->isReach(this->goal_, this->reachGoalDistance_, true) and (this->replan_ or this->trajectoryReady_)){
			cout << "\033[[AutoFlight]: Finishing entire path. Wait for new path. Press ENTER to Replan.\033[0m" << endl;
			this->replan_ = false;
			this->trajectoryReady_ = false;
			return;		
		}

		if (this->waypoints_.poses.size() != 0){
			if (not this->isGoalValid() and (this->replan_ or this->trajectoryReady_)){
				this->replan_ = false;
				this->trajectoryReady_ = false;
				cout << "\033[1;32m[AutoFlight]: Current goal is invalid. Need new path. Press ENTER to Replan.\033[0m" << endl;
				// this->explorationReplan_ = true;
				return;
			}
		}

		// if (this->reachExplorationGoal()){
		// 	this->replan_ = false;
		// 	this->trajectoryReady_ = false;
		// 	geometry_msgs::Quaternion quat = this->waypoints_.poses.back().pose.orientation;
		// 	double yaw = AutoFlight::rpy_from_quaternion(quat);
		// 	cout << "[AutoFlight]: Reach exploration goal. Rotate and replan..." << endl;
		// 	this->moveToOrientation(yaw, this->desiredAngularVel_);
		// 	cout << "[AutoFlight]: Finish rotation. Start to replan." << endl;
		// 	this->replan_ = true;
		// 	return;
		// }

		if (this->trajectoryReady_){
			if (not this->expPlanner_->isPosValid(this->trajectory_.at(this->trajectory_.getDuration()))){
				this->trajectoryReady_ = false;
				this->replan_ = false;
				this->stop();
				cout << "\033[1;32m[AutoFlight]: the goal of current local trajectory is not safe. Press ENTER to Replan.\033[0m" << endl;
				return;
			}

			if (this->hasCollision()){ // if trajectory not ready, do not replan
				this->replan_ = true;
				cout << "[AutoFlight]: Replan for collision." << endl;
				return;
			}

			// replan for dynamic obstacles
			if (this->computeExecutionDistance() >= 0.3 and this->hasDynamicCollision()){
			// if (this->hasDynamicObstacle()){
				this->replan_ = true;
				cout << "[AutoFlight]: Replan for dynamic obstacles." << endl;
				return;
			}

			if (this->computeExecutionDistance() >= 1.5 and AutoFlight::getPoseDistance(this->odom_.pose.pose, this->goal_.pose) >= 3){
				this->replan_ = true;
				cout << "[AutoFlight]: Regular replan." << endl;
				return;
			}

			if (this->computeExecutionDistance() >= 0.3 and this->replanForDynamicObstacle()){
				this->replan_ = true;
				cout << "[AutoFlight]: Regular replan for dynamic obstacles." << endl;
				return;
			}
		}	
	}

	void dynamicExploration::trajExeCB(const ros::TimerEvent&){
		if (this->trajectoryReady_){
			ros::Time currTime = ros::Time::now();
			double realTime = (currTime - this->trajStartTime_).toSec();
			this->trajTime_ = this->bsplineTraj_->getLinearReparamTime(realTime);
			double linearReparamFactor = this->bsplineTraj_->getLinearFactor();
			Eigen::Vector3d pos = this->trajectory_.at(this->trajTime_);
			Eigen::Vector3d vel = this->trajectory_.getDerivative().at(this->trajTime_) * linearReparamFactor;
			Eigen::Vector3d acc = this->trajectory_.getDerivative().getDerivative().at(this->trajTime_) * pow(linearReparamFactor, 2);
			double endTime = this->trajectory_.getDuration()/linearReparamFactor;

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
				target.yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
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

	void dynamicExploration::visCB(const ros::TimerEvent&){
		if (this->polyTrajMsg_.poses.size() != 0){
			this->polyTrajPub_.publish(this->polyTrajMsg_);
		}
		if (this->pwlTrajMsg_.poses.size() != 0){
			this->pwlTrajPub_.publish(this->pwlTrajMsg_);
		}
		if (this->bsplineTrajMsg_.poses.size() != 0){
			this->bsplineTrajPub_.publish(this->bsplineTrajMsg_);
		}
		if (this->inputTrajMsg_.poses.size() != 0){
			this->inputTrajPub_.publish(this->inputTrajMsg_);
		}
	}

	void dynamicExploration::freeMapCB(const ros::TimerEvent&){
		std::vector<onboardDetector::box3D> obstacles;
		std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> freeRegions;
		this->detector_->getObstacles(obstacles);
		double fov = 1.57;
		for (onboardDetector::box3D ob: obstacles){
			if (this->detector_->isObstacleInSensorRange(ob, fov)){
				Eigen::Vector3d lowerBound (ob.x-ob.x_width/2-0.3, ob.y-ob.y_width/2-0.3, ob.z);
				Eigen::Vector3d upperBound (ob.x+ob.x_width/2+0.3, ob.y+ob.y_width/2+0.3, ob.z+ob.z_width+0.3);
				freeRegions.push_back(std::make_pair(lowerBound, upperBound));
			}
		}
		this->map_->updateFreeRegions(freeRegions);
		this->map_->freeRegions(freeRegions);
	}

	void dynamicExploration::run(){
		cout << "\033[1;32m[AutoFlight]: Please double check all parameters. Then PRESS ENTER to continue or PRESS CTRL+C to stop.\033[0m" << endl;
		std::cin.clear();
		fflush(stdin);
		std::cin.get();
		this->takeoff();

		cout << "\033[1;32m[AutoFlight]: Takeoff succeed. Then PRESS ENTER to continue or PRESS CTRL+C to land.\033[0m" << endl;
		std::cin.clear();
		fflush(stdin);
		std::cin.get();

		int temp1 = system("mkdir ~/rosbag_exploration_info &");
		int temp2 = system("mv ~/rosbag_exploration_info/exploration_info.bag ~/rosbag_exploration_info/previous.bag &");
		int temp3 = system("rosbag record -O ~/rosbag_exploration_info/exploration_info.bag /camera/aligned_depth_to_color/image_raw_t /camera/color/image_raw_t /dynamic_map/inflated_voxel_map_t /onboard_detector/dynamic_bboxes /mavros/local_position/pose /dynamicExploration/bspline_trajectory /mavros/setpoint_position/local /tracking_controller/target_pose /dep/best_paths /dep/roadmap /dep/candidate_paths /dep/best_paths /dep/frontier_regions /dynamic_map/2D_occupancy_map __name:=exploration_bag_info &");
		if (temp1==-1 or temp2==-1 or temp3==-1){
			cout << "[AutoFlight]: Recording fails." << endl;
		}

		this->initExplore();

		cout << "\033[1;32m[AutoFlight]: PRESS ENTER to Start Planning.\033[0m" << endl;
		std::cin.clear();
		fflush(stdin);
		std::cin.get();

		this->registerCallback();
	}

	void dynamicExploration::initExplore(){
		// set start region to be free
		// Eigen::Vector3d range (2.0, 2.0, 1.0);
		Eigen::Vector3d startPos (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		Eigen::Vector3d c1 = startPos - this->freeRange_;
		Eigen::Vector3d c2 = startPos + this->freeRange_;
		this->map_->freeRegion(c1, c2);
		cout << "[AutoFlight]: Robot nearby region is set to free. Range: " << this->freeRange_.transpose() << endl;

		if (this->initialScan_){
			cout << "[AutoFlight]: Start initial scan..." << endl;
			this->moveToOrientation(-PI_const/2, this->desiredAngularVel_);
			cout << "\033[1;32m[AutoFlight]: Press ENTER to continue next 90 degree.\033[0m" << endl;
			std::cin.clear();
			fflush(stdin);
			std::cin.get();
						
			this->moveToOrientation(-PI_const, this->desiredAngularVel_);
			cout << "\033[1;32m[AutoFlight]: Press ENTER to continue next 90 degree.\033[0m" << endl;
			std::cin.clear();
			fflush(stdin);
			std::cin.get();

			this->moveToOrientation(PI_const/2, this->desiredAngularVel_);
			cout << "\033[1;32m[AutoFlight]: Press ENTER to continue next 90 degree.\033[0m" << endl;
			std::cin.clear();
			fflush(stdin);
			std::cin.get();
			
			this->moveToOrientation(0, this->desiredAngularVel_);
			cout << "[AutoFlight]: End initial scan." << endl; 
		}		
	}

	void dynamicExploration::getStartEndConditions(std::vector<Eigen::Vector3d>& startEndConditions){
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

	bool dynamicExploration::hasCollision(){
		if (this->trajectoryReady_){
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

	bool dynamicExploration::hasDynamicCollision(){
		if (this->trajectoryReady_){
			std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
			if (this->useFakeDetector_){
				this->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
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
		return false;
	}

	void dynamicExploration::exploreReplan(){
		// set start region to be free
		// Eigen::Vector3d range (2.0, 2.0, 1.0);
		// Eigen::Vector3d startPos (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		// Eigen::Vector3d c1 = startPos - range;
		// Eigen::Vector3d c2 = startPos + range;
		// this->map_->freeRegion(c1, c2);
		// cout << "[AutoFlight]: Robot nearby region is set to free. Range: " << range.transpose() << endl;

		// if (this->initialScan_){
		// 	cout << "[AutoFlight]: Start initial scan..." << endl;
		// 	this->moveToOrientation(-PI_const/2, this->desiredAngularVel_);
		// 	cout << "[AutoFlight]: Press ENTER to continue next 90 degree." << endl;
		// 	std::cin.clear();
		// 	fflush(stdin);
		// 	std::cin.get();
						
		// 	this->moveToOrientation(-PI_const, this->desiredAngularVel_);
		// 	cout << "[AutoFlight]: Press ENTER to continue next 90 degree." << endl;
		// 	std::cin.clear();
		// 	fflush(stdin);
		// 	std::cin.get();

		// 	this->moveToOrientation(PI_const/2, this->desiredAngularVel_);
		// 	cout << "[AutoFlight]: Press ENTER to continue next 90 degree." << endl;
		// 	std::cin.clear();
		// 	fflush(stdin);
		// 	std::cin.get();
			
		// 	this->moveToOrientation(0, this->desiredAngularVel_);
		// 	cout << "[AutoFlight]: End initial scan." << endl; 
		// }
		while (ros::ok()){
			this->expPlanner_->setMap(this->map_);
			ros::Time startTime = ros::Time::now();
			bool replanSuccess = this->expPlanner_->makePlan();
			if (replanSuccess){
				this->waypoints_ = this->expPlanner_->getBestPath();
				this->newWaypoints_ = true;
				this->waypointIdx_ = 1;
			}
			ros::Time endTime = ros::Time::now();
			std::cin.clear();
			fflush(stdin);
			std::cin.get();		
			cout << "[AutoFlight]: DEP planning time: " << (endTime - startTime).toSec() << "s." << endl;

		}
	}

	double dynamicExploration::computeExecutionDistance(){
		if (this->trajectoryReady_ and not this->replan_){
			Eigen::Vector3d prevP, currP;
			bool firstTime = true;
			double totalDistance = 0.0;
			double remainDistance = 0.0;
			for (double t=0.0; t<=this->trajectory_.getDuration(); t+=0.1){
				currP = this->trajectory_.at(t);
				if (firstTime){
					firstTime = false;
				}
				else{
					if (t <= this->trajTime_){
						totalDistance += (currP - prevP).norm();
					}
					else{
						remainDistance += (currP - prevP).norm();
					}
				}
				prevP = currP;
			}
			if (remainDistance <= 1.0){ // no replan when less than 1m
				return -1.0;
			}
			return totalDistance;
		}
		return -1.0;
	}

	bool dynamicExploration::replanForDynamicObstacle(){
		ros::Time currTime = ros::Time::now();
		std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
		if (this->useFakeDetector_){
			this->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
		}
		else{ 
			this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
		}

		bool replan = false;
		bool hasDynamicObstacle = (obstaclesPos.size() != 0);
		if (hasDynamicObstacle){
			double timePassed = (currTime - this->lastDynamicObstacleTime_).toSec();
			if (timePassed >= this->replanTimeForDynamicObstacle_){
				replan = true;
				this->lastDynamicObstacleTime_ = currTime;
			}
		}
		return replan;
	}

	bool dynamicExploration::reachExplorationGoal(){
		if (this->waypoints_.poses.size() == 0) return false;
		double distThresh = 0.1;
		double yawDiffThresh = 0.1;
		Eigen::Vector3d currPos (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		Eigen::Vector3d targetPos (this->waypoints_.poses.back().pose.position.x, this->waypoints_.poses.back().pose.position.y, this->waypoints_.poses.back().pose.position.z);
		double yawTarget = AutoFlight::rpy_from_quaternion(this->waypoints_.poses.back().pose.orientation); 
		double currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		if ((currPos - targetPos).norm() <= distThresh and std::abs(currYaw - yawTarget) >= yawDiffThresh){
			return true;
		}
		return false;
	}

	bool dynamicExploration::isGoalValid(){
		Eigen::Vector3d pGoal (this->goal_.pose.position.x, this->goal_.pose.position.y, this->goal_.pose.position.z);
		if (this->map_->isInflatedOccupied(pGoal)){
			return false;
		}
		else{
			return true;
		}
	}

	nav_msgs::Path dynamicExploration::getCurrentTraj(double dt){
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

	nav_msgs::Path dynamicExploration::getRestGlobalPath(){
		nav_msgs::Path currPath;

		int nextIdx = this->waypoints_.poses.size()-1;
		Eigen::Vector3d pCurr (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		double minDist = std::numeric_limits<double>::infinity();
		for (size_t i=0; i<this->waypoints_.poses.size()-1; ++i){
			geometry_msgs::PoseStamped ps = this->waypoints_.poses[i];
			Eigen::Vector3d pEig (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
			Eigen::Vector3d pDiff = pCurr - pEig;

			geometry_msgs::PoseStamped psNext = this->waypoints_.poses[i+1];
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
		for (size_t i=nextIdx; i<this->waypoints_.poses.size(); ++i){
			currPath.poses.push_back(this->waypoints_.poses[i]);
		}
		return currPath;		
	}

	nav_msgs::Path dynamicExploration::getRestGlobalPath(const Eigen::Vector3d& pos){
		nav_msgs::Path currPath;

		int nextIdx = this->waypoints_.poses.size()-1;
		Eigen::Vector3d pCurr = pos;
		double minDist = std::numeric_limits<double>::infinity();
		for (size_t i=0; i<this->waypoints_.poses.size()-1; ++i){
			geometry_msgs::PoseStamped ps = this->waypoints_.poses[i];
			Eigen::Vector3d pEig (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
			Eigen::Vector3d pDiff = pCurr - pEig;

			geometry_msgs::PoseStamped psNext = this->waypoints_.poses[i+1];
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
		for (size_t i=nextIdx; i<this->waypoints_.poses.size(); ++i){
			currPath.poses.push_back(this->waypoints_.poses[i]);
		}
		return currPath;		
	}

	nav_msgs::Path dynamicExploration::getRestGlobalPath(const Eigen::Vector3d& pos, double yaw){
		nav_msgs::Path currPath;

		int nextIdx = this->waypoints_.poses.size()-1;
		Eigen::Vector3d pCurr = pos;
		Eigen::Vector3d direction (cos(yaw), sin(yaw), 0);
		double minDist = std::numeric_limits<double>::infinity();
		for (size_t i=0; i<this->waypoints_.poses.size()-1; ++i){
			geometry_msgs::PoseStamped ps = this->waypoints_.poses[i];
			Eigen::Vector3d pEig (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);

			geometry_msgs::PoseStamped psNext = this->waypoints_.poses[i+1];
			Eigen::Vector3d pEigNext (psNext.pose.position.x, psNext.pose.position.y, psNext.pose.position.z);
			Eigen::Vector3d diffToNext = pEigNext - pEig;
			double dist = (pEig - pCurr).norm();
			if (trajPlanner::angleBetweenVectors(diffToNext, direction) > PI_const*3.0/4.0){
				if (dist < minDist){
					nextIdx = i;
					minDist = dist;
				}
			}
		}


		geometry_msgs::PoseStamped psCurr;
		psCurr.pose = this->odom_.pose.pose;
		currPath.poses.push_back(psCurr);
		for (size_t i=nextIdx; i<this->waypoints_.poses.size(); ++i){
			currPath.poses.push_back(this->waypoints_.poses[i]);
		}
		return currPath;		
	}

	void dynamicExploration::getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclesPos, std::vector<Eigen::Vector3d>& obstaclesVel, std::vector<Eigen::Vector3d>& obstaclesSize){
		std::vector<onboardDetector::box3D> obstacles;
		this->detector_->getObstaclesInSensorRange(PI_const, obstacles);
		for (onboardDetector::box3D ob : obstacles){
			Eigen::Vector3d pos (ob.x, ob.y, ob.z);
			Eigen::Vector3d vel (ob.Vx, ob.Vy, 0.0);
			Eigen::Vector3d size (ob.x_width, ob.y_width, ob.z_width);
			obstaclesPos.push_back(pos);
			obstaclesVel.push_back(vel);
			obstaclesSize.push_back(size);
		}
	}


	void dynamicExploration::waitTime(double time){
		ros::Rate r (30);
		ros::Time startTime = ros::Time::now();
		ros::Time currTime = ros::Time::now();
		double passtime = 0.0;
		while (ros::ok() and passtime < time){
			currTime = ros::Time::now();
			passtime = (currTime - startTime).toSec();
			ros::spinOnce();
			r.sleep();
		}
	}
}