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
		cout << "in regisuter callback" << endl;
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
	}

	void dynamicExploration::registerPub(){
		this->inputTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicExploration/input_trajectory", 10);
	}

	void dynamicExploration::plannerCB(const ros::TimerEvent&){
		// cout << "in planner callback" << endl;
		if (this->newWaypoints_){
			cout << "start generating poly traj" << endl;
			nav_msgs::Path inputTraj;
			std::vector<Eigen::Vector3d> startEndConditions;
			this->getStartEndConditions(startEndConditions); 
			double finalTime; // final time for bspline trajectory
			double initTs = this->bsplineTraj_->getInitTs();

			this->polyTraj_->updatePath(this->waypoints_);
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
				if (satisfyDistanceCheck) break;
				dtTemp *= 0.8;
			}
			inputTraj = adjustedInputPolyTraj;
			this->inputTrajMsg_ = inputTraj;
			finalTime = finalTimeTemp;
			startEndConditions[1] = this->polyTraj_->getVel(finalTime);
			startEndConditions[3] = this->polyTraj_->getAcc(finalTime);	
			this->newWaypoints_ = false;
			cout << "poly traj generated" << endl;

		}
	}

	void dynamicExploration::replanCheckCB(const ros::TimerEvent&){
		// cout << "int replan check callback" << endl;
	}

	void dynamicExploration::trajExeCB(const ros::TimerEvent&){
		// cout << "in traj exe callback" << endl;
	}

	void dynamicExploration::visCB(const ros::TimerEvent&){
		if (this->inputTrajMsg_.poses.size() != 0){
			this->inputTrajPub_.publish(this->inputTrajMsg_);
		}
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
}