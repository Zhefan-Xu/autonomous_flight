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
	}

	void dynamicExploration::initModules(){
		// initialize map
		if (this->useFakeDetector_){
			// initialize fake detector
			this->detector_.reset(new onboardVision::fakeDetector (this->nh_));	
		}
		this->map_.reset(new mapManager::dynamicMap (this->nh_));

		// initialize exploration planner
		this->expPlanner_.reset(new globalPlanner::DEP (this->nh_));
		this->expPlanner_->setMap(this->map_);

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

		// planner callback
		this->plannerTimer_ = this->nh_.createTimer(ros::Duration(0.02), &dynamicExploration::plannerCB, this);

		// replan check timer
		this->replanCheckTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dynamicExploration::replanCheckCB, this);
		
		// trajectory execution callback
		this->trajExeTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dynamicExploration::trajExeCB, this);
	
		// visualization execution callabck
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.033), &dynamicExploration::visCB, this);

		if (this->useFakeDetector_){
			// free map callback
			this->freeMapTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dynamicExploration::freeMapCB, this);
		}
	}

	void dynamicExploration::registerPub(){
		this->polyTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicExploration/poly_traj", 10);
		this->pwlTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicExploration/pwl_trajectory", 10);
		this->bsplineTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicExploration/bspline_trajectory", 10);
		this->inputTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicExploration/input_trajectory", 10);
	}

	void dynamicExploration::plannerCB(const ros::TimerEvent&){
		// cout << "in planner callback" << endl;

		if (this->replan_){
			// if (this->newWaypoints_){
			cout << "new waypoints size: " << this->waypoints_.poses.size() << endl;
			cout << "start generating poly traj" << endl;
			nav_msgs::Path inputTraj;
			std::vector<Eigen::Vector3d> startEndConditions;
			this->getStartEndConditions(startEndConditions); 
			double finalTime; // final time for bspline trajectory
			double initTs = this->bsplineTraj_->getInitTs();
			// if (not this->trajectoryReady_){
			nav_msgs::Path latestGLobalPath = this->getRestGlobalPath();
			this->polyTraj_->updatePath(latestGLobalPath);
			this->polyTraj_->makePlan(this->polyTrajMsg_);
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
				// if (adjustedInputPolyTraj.poses.size() >= 4 and satisfyDistanceCheck) break;
				if (satisfyDistanceCheck) break;
				dtTemp *= 0.8;
			}
			inputTraj = adjustedInputPolyTraj;
			finalTime = finalTimeTemp;
			startEndConditions[1] = this->polyTraj_->getVel(finalTime);
			startEndConditions[3] = this->polyTraj_->getAcc(finalTime);	
			
			cout << "poly traj generated" << endl;

			// }
			// else{
			// 	Eigen::Vector3d bsplineLastPos = this->trajectory_.at(this->trajectory_.getDuration());
			// 	geometry_msgs::PoseStamped lastPs; lastPs.pose.position.x = bsplineLastPos(0); lastPs.pose.position.y = bsplineLastPos(1); lastPs.pose.position.z = bsplineLastPos(2);
			// 	Eigen::Vector3d goalPos (this->goal_.pose.position.x, this->goal_.pose.position.y, this->goal_.pose.position.z);
			// 	// check the distance between last point and the goal position
			// 	if ((bsplineLastPos - goalPos).norm() >= 0.2){ // use polynomial trajectory to make the rest of the trajectory
			// 		nav_msgs::Path waypoints, polyTrajTemp;
			// 		// waypoints.poses = std::vector<geometry_msgs::PoseStamped>{lastPs, this->goal_}; // TODO!!!!!!!!!!!
			// 		waypoints = this->getRestGlobalPath(bsplineLastPos);
			// 		std::vector<Eigen::Vector3d> polyStartEndConditions;
			// 		Eigen::Vector3d polyStartVel = this->trajectory_.getDerivative().at(this->trajectory_.getDuration());
			// 		Eigen::Vector3d polyEndVel (0.0, 0.0, 0.0);
			// 		Eigen::Vector3d polyStartAcc = this->trajectory_.getDerivative().getDerivative().at(this->trajectory_.getDuration());
			// 		Eigen::Vector3d polyEndAcc (0.0, 0.0, 0.0);
			// 		polyStartEndConditions.push_back(polyStartVel);
			// 		polyStartEndConditions.push_back(polyEndVel);
			// 		polyStartEndConditions.push_back(polyStartAcc);
			// 		polyStartEndConditions.push_back(polyEndAcc);
			// 		this->polyTraj_->updatePath(waypoints, polyStartEndConditions);
			// 		this->polyTraj_->makePlan(false); // no corridor constraint
					
			// 		nav_msgs::Path adjustedInputCombinedTraj;
			// 		bool satisfyDistanceCheck = false;
			// 		double dtTemp = initTs;
			// 		double finalTimeTemp;
			// 		ros::Time startTime = ros::Time::now();
			// 		ros::Time currTime;
			// 		while (ros::ok()){
			// 			currTime = ros::Time::now();
			// 			if ((currTime - startTime).toSec() >= 0.05){
			// 				cout << "[AutoFlight]: Exceed path check time. Use the best." << endl;
			// 				break;
			// 			}							
			// 			nav_msgs::Path inputRestTraj = this->getCurrentTraj(dtTemp);
			// 			cout << "size of rest traj: " << inputRestTraj.poses.size() << endl;
			// 			for (int i=0; i<inputRestTraj.poses.size() and i<10; ++i){
			// 				geometry_msgs::PoseStamped p1 = inputRestTraj.poses[i];
			// 				Eigen::Vector3d p1eig (p1.pose.position.x, p1.pose.position.y, p1.pose.position.z);
			// 				cout << "rest p1: " << p1eig.transpose() << endl;
			// 			}
			// 			nav_msgs::Path inputPolyTraj = this->polyTraj_->getTrajectory(dtTemp);
			// 			cout << "size of poly traj: " << inputPolyTraj.poses.size() << endl;
			// 			for (int i=0; i<inputPolyTraj.poses.size()-1 and i<10; ++i){
			// 				geometry_msgs::PoseStamped p1 = inputPolyTraj.poses[i];
			// 				geometry_msgs::PoseStamped p2 = inputPolyTraj.poses[i+1];
			// 				Eigen::Vector3d p1eig = Eigen::Vector3d (p1.pose.position.x, p1.pose.position.y, p1.pose.position.z);
			// 				Eigen::Vector3d p2eig = Eigen::Vector3d (p2.pose.position.x, p2.pose.position.y, p2.pose.position.z);
			// 				// cout << "distance: " << (p1eig - p2eig).norm() << endl;
			// 				cout << "p1: " << p1eig.transpose() << endl;
			// 			}
			// 			nav_msgs::Path inputCombinedTraj;
			// 			inputCombinedTraj.poses = inputRestTraj.poses;
			// 			for (size_t i=1; i<inputPolyTraj.poses.size(); ++i){
			// 				inputCombinedTraj.poses.push_back(inputPolyTraj.poses[i]);
			// 			}
						
			// 			satisfyDistanceCheck = this->bsplineTraj_->inputPathCheck(inputCombinedTraj, adjustedInputCombinedTraj, dtTemp, finalTimeTemp);
			// 			// if (adjustedInputCombinedTraj.poses.size() >= 4 and satisfyDistanceCheck) break;
			// 			if (satisfyDistanceCheck) break;
						
			// 			dtTemp *= 0.8; // magic number 0.8
			// 		}
			// 		cout << "adjusted input combiend traj size: " << adjustedInputCombinedTraj.poses.size() << endl;
			// 		inputTraj = adjustedInputCombinedTraj;
			// 		finalTime = finalTimeTemp - this->trajectory_.getDuration(); // need to subtract prev time since it is combined trajectory
			// 		startEndConditions[1] = this->polyTraj_->getVel(finalTime);
			// 		startEndConditions[3] = this->polyTraj_->getAcc(finalTime);
			// 		cout <<  "in previous replan 1" << endl;
			// 	}
			// 	else{
			// 		nav_msgs::Path adjustedInputRestTraj;
			// 		bool satisfyDistanceCheck = false;
			// 		double dtTemp = initTs;
			// 		double finalTimeTemp;
			// 		ros::Time startTime = ros::Time::now();
			// 		ros::Time currTime;
			// 		while (ros::ok()){
			// 			currTime = ros::Time::now();
			// 			if ((currTime - startTime).toSec() >= 0.05){
			// 				cout << "[AutoFlight]: Exceed path check time. Use the best." << endl;
			// 				break;
			// 			}
			// 			nav_msgs::Path inputRestTraj = this->getCurrentTraj(dtTemp);
			// 			satisfyDistanceCheck = this->bsplineTraj_->inputPathCheck(inputRestTraj, adjustedInputRestTraj, dtTemp, finalTimeTemp);
			// 			// if (adjustedInputRestTraj.poses.size() >= 4 and satisfyDistanceCheck) break;
			// 			if (satisfyDistanceCheck) break;
			// 			dtTemp *= 0.8;
			// 		}
			// 		inputTraj = adjustedInputRestTraj;
			// 		cout <<  "in previous replan 2" << endl;
			// 	}
			// }
			this->newWaypoints_ = false;
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
					this->trajectoryReady_ = true;
					this->replan_ = false;
					cout << "[AutoFlight]: Trajectory generated successfully." << endl;
				}
				else{
					// if the current trajectory is still valid, then just ignore this iteration
					// if the current trajectory/or new goal point is assigned is not valid, then just stop
					if (this->hasCollision() or this->hasDynamicCollision()){
						this->trajectoryReady_ = false;
						this->stop();
						cout << "[AutoFlight]: Stop!!! Trajectory generation fails." << endl;
					}
					else{
						if (this->trajectoryReady_){
							cout << "[AutoFlight]: Trajectory fail. Use trajectory from previous iteration." << endl;
						}
						else{
							cout << "[AutoFlight]: Unable to generate a feasible trajectory." << endl;
						}
						this->replan_ = false;
					}
				}
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
			this->moveToOrientation(yaw, this->desiredAngularVel_);
			this->replan_ = true;
			this->newWaypoints_ = false;
			cout << "[AutoFlight]: Replan for new waypoints." << endl; 
			return;
		}


		if (this->trajectoryReady_){
			if (this->hasCollision()){ // if trajectory not ready, do not replan
				this->replan_ = true;
				cout << "[AutoFlight]: Replan for collision." << endl;
				return;
			}

			// replan for dynamic obstacles
			if (this->hasDynamicCollision()){
				this->replan_ = true;
				cout << "[AutoFlight]: Replan for dynamic obstacles." << endl;
				return;
			}

			if (this->computeExecutionDistance() >= 3.0){
				this->replan_ = true;
				cout << "[AutoFlight]: Regular replan." << endl;
				return;
			}

			// if (this->replanForDynamicObstacle()){
			// 	this->replan_ = true;
			// 	cout << "[AutoFlight]: Regular replan for dynamic obstacles." << endl;
			// 	return;
			// }
		}	}

	void dynamicExploration::trajExeCB(const ros::TimerEvent&){
		if (this->trajectoryReady_){
			ros::Time currTime = ros::Time::now();
			double trajTime = (currTime - this->trajStartTime_).toSec();
			this->trajTime_ = this->bsplineTraj_->getLinearReparamTime(trajTime);
			double linearReparamFactor = this->bsplineTraj_->getLinearFactor();
			Eigen::Vector3d pos = this->trajectory_.at(this->trajTime_);
			Eigen::Vector3d vel = this->trajectory_.getDerivative().at(this->trajTime_) * linearReparamFactor;
			Eigen::Vector3d acc = this->trajectory_.getDerivative().getDerivative().at(this->trajTime_) * pow(linearReparamFactor, 2);

			
			tracking_controller::Target target;
			// if (this->noYawTurning_ or not this->useYawControl_){
			// 	target.yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
			// }
			// else{
				target.yaw = atan2(vel(1), vel(0));
				// cout << "current vel: " << vel.transpose() << endl;
			// }
			if (std::abs(this->trajTime_ - this->trajectory_.getDuration()) <= 0.3 or this->trajTime_ > this->trajectory_.getDuration()){ // zero vel and zero acc if close to
				vel *= 0;
				acc *= 0;
				target.yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
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
		onboard_vision::ObstacleList msg;
		std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> freeRegions;
		this->detector_->getObstacles(msg);
		for (onboard_vision::Obstacle ob: msg.obstacles){
			Eigen::Vector3d lowerBound (ob.px-ob.xsize/2-0.3, ob.py-ob.ysize/2-0.3, ob.pz);
			Eigen::Vector3d upperBound (ob.px+ob.xsize/2+0.3, ob.py+ob.ysize/2+0.3, ob.pz+ob.zsize+0.2);
			freeRegions.push_back(std::make_pair(lowerBound, upperBound));
		}

		this->map_->updateFreeRegions(freeRegions);	
		this->map_->freeHistRegions();
	}

	void dynamicExploration::run(){
		this->takeoff();
		this->registerCallback();
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
		cout << "in replan function" << endl;
		while (ros::ok()){
			cout << "updating map..." << endl;
			this->expPlanner_->setMap(this->map_);
			cout << "start planning." << endl;
			ros::Time startTime = ros::Time::now();
			bool replanSuccess = this->expPlanner_->makePlan();
			if (replanSuccess){
				this->waypoints_ = this->expPlanner_->getBestPath();
				this->newWaypoints_ = true;
			}
			ros::Time endTime = ros::Time::now();
			cout << "replan success: " << replanSuccess << endl;
			cout << "planning time: " << (endTime - startTime).toSec() << "s." << endl;
			cout << "end planning." << endl;


			cout << "PRESS ENTER to Replan." << endl;
			std::cin.clear();
			fflush(stdin);
			std::cin.get();					
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

	double dynamicExploration::computeExecutionDistance(){
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

	void dynamicExploration::getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclesPos, std::vector<Eigen::Vector3d>& obstaclesVel, std::vector<Eigen::Vector3d>& obstaclesSize){
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