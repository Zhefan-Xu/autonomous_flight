/*
	FILE: inspeciton.cpp
	-----------------------------
	Implementation of autonomous inspection 
*/

#include <autonomous_flight/px4/inspection.h>

namespace AutoFlight{
	inspector::inspector(const ros::NodeHandle& nh) : flightBase(nh){
		this->loadParam();
		this->initPlanner();
		this->mapSub_ = this->nh_.subscribe("/octomap_full", 1, &inspector::mapCB, this);
		cout << "init succeed" << endl;
	}

	void inspector::loadParam(){
		// Collision box
		if (not this->nh_.getParam("collision_box", this->collisionBox_)){
			this->collisionBox_ = std::vector<double> {0.75, 0.75, 0.3};
			cout << "[AutoFlight]: No Collision box param. Use default {0.75, 0.75, 0.3}." << endl;
		}

		// Safe distance
		if (not this->nh_.getParam("safe_distance", this->safeDist_)){
			this->safeDist_ = 1.0;
			cout << "[AutoFlight]: No safe distance param. Use default 1.0m." << endl;
		}
	}

	void inspector::initPlanner(){
		this->pwlPlanner_ = new trajPlanner::pwlTraj (this->nh_);
	}

	void inspector::run(){
		this->lookAround();

		// forward to surface
		// this->forward();

		// check surroundings and dimensions of the surface
		this->checkSurroundings();

		// inspect the surface by zig-zag path
		this->inspect();
	
		// go back to the start position
		this->backward();
	}

	void inspector::lookAround(){
		cout << "Look around" << endl;
		nav_msgs::Path lookAroundPath;
		
		geometry_msgs::PoseStamped ps;
		ps.pose = this->odom_.pose.pose;
		double currYaw = trajPlanner::rpy_from_quaternion(ps.pose.orientation);
		double targetYaw1 = currYaw + M_PI/2;
		double targetYaw2 = currYaw - M_PI/2;
		geometry_msgs::PoseStamped ps1, ps2;
		ps1.pose.position = ps.pose.position;
		ps2.pose.position = ps.pose.position;
		ps1.pose.orientation = AutoFlight::quaternion_from_rpy(0, 0, targetYaw1);
		ps2.pose.orientation = AutoFlight::quaternion_from_rpy(0, 0, targetYaw2);
		std::vector<geometry_msgs::PoseStamped> lookAroundPathVec = std::vector<geometry_msgs::PoseStamped> {ps, ps1, ps, ps2, ps};
		lookAroundPath.poses = lookAroundPathVec;
		this->pwlPlanner_->updatePath(lookAroundPath, true);

		double t = 0.0;
		ros::Rate r (1.0/this->sampleTime_);
		ros::Time tStart = ros::Time::now();
		while (ros::ok() and (not this->isReach(ps) or t <= this->pwlPlanner_->getDuration())){
			ros::Time tCurr = ros::Time::now();
			t = (tCurr - tStart).toSec();
			cout << "t" << t << endl;
			if (t > this->pwlPlanner_->getDuration()){
				t = this->pwlPlanner_->getDuration();
			}
			geometry_msgs::PoseStamped targetPose = this->pwlPlanner_->getPose(t);
			cout << "t" << t << endl;
			cout << targetPose.pose.position.x << " " << targetPose.pose.position.y << endl;
			this->updateTarget(targetPose);
			r.sleep();
		}
		cout << "Finish lookaround" << endl;
	}


	void inspector::forward(){
		// determine the goal position
		// 1. check collision in the line of x direction until collision happens
		// 2. go back to the position to safe dist to the collision position
		// 3. if it is less than safe dist already, stay at the same location

		ros::Time startTime, currTime;
		double t = 0.0;
		nav_msgs::Path forwardPath;
		bool firstTime = true;

		ros::Rate r (1.0/this->sampleTime_);
		while (ros::ok()){
			ros::Time t1 = ros::Time::now();
			ros::Time t2 = ros::Time::now();
			cout << "========new path=========" << endl;
			forwardPath = this->getForwardPath();
			ros::Time t3 = ros::Time::now();
			this->pwlPlanner_->updatePath(forwardPath);
			ros::Time t4 = ros::Time::now();
			geometry_msgs::PoseStamped ps = this->pwlPlanner_->getPose(this->sampleTime_);
			ros::Time t5 = ros::Time::now(); 
			this->updateTarget(ps);
			ros::Time t6 = ros::Time::now();
			// cout << "1: " << (t2 - t1).toSec() << endl;
			// cout << "2: " << (t3 - t2).toSec() << endl;
			// cout << "3: " << (t4 - t3).toSec() << endl;
			// cout << "4: " << (t5 - t4).toSec() << endl;
			// cout << "5: " << (t6 - t5).toSec() << endl;
			r.sleep();
		}
	}

	void inspector::checkSurroundings(){

	}

	void inspector::inspect(){

	}

	void inspector::backward(){

	}

	void inspector::mapCB(const octomap_msgs::Octomap &msg){
    	octomap::OcTree* treePtr = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(msg));
   	 	this->map_ = std::shared_ptr<octomap::OcTree>(treePtr);
   	 	this->mapRes_ = this->map_->getResolution();
   	 	this->setSurroundingFree(this->getPoint3dPos());
   	 	// cout << "here" << endl;
	}

	geometry_msgs::PoseStamped inspector::getForwardGoal(){
		octomap::point3d p = this->getPoint3dPos();
	
		// cast a ray along the x direction and check collision
		float res = this->mapRes_;
		octomap::point3d pForward = p;
		while (ros::ok() and not this->checkCollision(pForward)){
			pForward.x() += res;
			cout << res << endl;
			cout << pForward << endl;
		}
		octomap::point3d pGoal = pForward;
		pGoal.x() -= res;
		pGoal.x() -= this->safeDist_;

		geometry_msgs::PoseStamped ps;
		ps.header.frame_id = "map";
		ps.header.stamp = ros::Time::now();
		ps.pose.position.x = pGoal.x();
		ps.pose.position.y = pGoal.y();
		ps.pose.position.z = pGoal.z();
		ps.pose.orientation = this->odom_.pose.pose.orientation;
		return ps;
	}

	nav_msgs::Path inspector::getForwardPath(){
		geometry_msgs::PoseStamped goalPs = this->getForwardGoal();
		geometry_msgs::PoseStamped startPs; 
		startPs.pose = this->odom_.pose.pose;
		std::vector<geometry_msgs::PoseStamped> forwardPathVec;
		forwardPathVec.push_back(startPs);
		forwardPathVec.push_back(goalPs);
		nav_msgs::Path forwardPath;
		forwardPath.header.frame_id = "map";
		forwardPath.header.stamp = ros::Time::now();
		forwardPath.poses = forwardPathVec;
		return forwardPath;
	}

	octomap::point3d inspector::getPoint3dPos(){
		float x, y, z;
		x = this->odom_.pose.pose.position.x;
		y = this->odom_.pose.pose.position.y;
		z = this->odom_.pose.pose.position.z;
		return octomap::point3d (x, y, z);
	}

	bool inspector::checkCollision(const octomap::point3d &p, bool ignoreUnknown){
		double xmin, xmax, ymin, ymax, zmin, zmax; // bounding box for collision checking
		xmin = p.x() - this->collisionBox_[0]/2.0; xmax = p.x() + this->collisionBox_[0]/2.0;
		ymin = p.y() - this->collisionBox_[1]/2.0; ymax = p.y() + this->collisionBox_[1]/2.0;
		zmin = p.z() - this->collisionBox_[2]/2.0; zmax = p.z() + this->collisionBox_[2]/2.0;

		int xNum = (xmax - xmin)/this->mapRes_;
		int yNum = (ymax - ymin)/this->mapRes_;
		int zNum = (zmax - zmin)/this->mapRes_;

		int xID, yID, zID;
		for (xID=0; xID<=xNum; ++xID){
			for (yID=0; yID<=yNum; ++yID){
				for (zID=0; zID<=zNum; ++zID){
					if (this->checkCollisionPoint(octomap::point3d(xmin+xID*this->mapRes_, ymin+yID*this->mapRes_, zmin+zID*this->mapRes_), ignoreUnknown)){
						return true;
					}
				}
			}
		}
		return false;
	}

	bool inspector::checkCollisionPoint(const octomap::point3d &p, bool ignoreUnknown){
		octomap::OcTreeNode* nptr = this->map_->search(p);
		if (nptr == NULL){
			if (not ignoreUnknown){
				return true;
			}
			else{
				return false;
			}
		}
		return this->map_->isNodeOccupied(nptr);
	}

	void inspector::setSurroundingFree(const octomap::point3d& p){
		const float logOddsFree = octomap::logodds(0.1);
		double xmin, xmax, ymin, ymax, zmin, zmax; // bounding box for collision checking
		xmin = p.x() - this->collisionBox_[0]/2; xmax = p.x() + this->collisionBox_[0]/2;
		ymin = p.y() - this->collisionBox_[1]/2; ymax = p.y() + this->collisionBox_[1]/2;
		zmin = p.z() - this->collisionBox_[2]/2; zmax = p.z() + this->collisionBox_[2]/2;

		int xNum = (xmax - xmin)/this->mapRes_;
		int yNum = (ymax - ymin)/this->mapRes_;
		int zNum = (zmax - zmin)/this->mapRes_;

		int xID, yID, zID;
		for (xID=0; xID<=xNum; ++xID){
			for (yID=0; yID<=yNum; ++yID){
				for (zID=0; zID<=zNum; ++zID){
					octomap::point3d pCheck (xmin+xID*this->mapRes_, ymin+yID*this->mapRes_, zmin+zID*this->mapRes_);
					octomap::OcTreeNode* nptr = this->map_->search(pCheck);
					if (nptr == NULL){
						this->map_->setNodeValue(pCheck, logOddsFree);
					}
				}
			}
		}		
	}
}