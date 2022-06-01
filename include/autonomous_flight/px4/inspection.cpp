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
			
		// visualization
		this->targetVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/inspection_target", 1);
		this->targetVisWorker_ = std::thread(&inspector::publishTargetVis, this);
		this->targetVisWorker_.detach();
		cout << "init succeed" << endl;
	}

	void inspector::loadParam(){
		// Collision box
		if (not this->nh_.getParam("collision_box", this->collisionBox_)){
			this->collisionBox_ = std::vector<double> {0.75, 0.75, 0.3};
			cout << "[AutoFlight]: No Collision box param. Use default {0.75, 0.75, 0.3}." << endl;
		}
		else{
			cout << "[AutoFlight]: Collision Box: [" << this->collisionBox_[0] 
				 << ", " << this->collisionBox_[1]
			 	 << ", " << this->collisionBox_[2] << "]" << endl; 
		}

		// Safe distance
		if (not this->nh_.getParam("safe_distance", this->safeDist_)){
			this->safeDist_ = 1.0;
			cout << "[AutoFlight]: No safe distance param. Use default 1.0m." << endl;
		}
		else{
			cout << "[AutoFlight]: Safe distance to wall is set to: " << this->safeDist_ << "m." << endl;
		}

		// min target area
		if (not this->nh_.getParam("min_target_area", this->minTargetArea_)){
			this->minTargetArea_ = 10;
			cout << "[AutoFlight]: No min target area for inspection. Use default: 10.0m^2" << endl;
		}
		else{
			cout << "[AutoFlight]: Min target area for inspection: " << this->minTargetArea_ << "m" << endl;
		}

		// max height
		if (not this->nh_.getParam("max_inspection_target_height", this->maxTargetHgt_)){
			this->maxTargetHgt_ = 3.0;
			cout << "[AutoFlight]: No max height for inspection. Use default: 3.0m" << endl;
		}
		else{
			cout << "[AutoFlight]: Max height for inspection: " << this->maxTargetHgt_ << "m" << endl;
		}


		// max width
		if (not this->nh_.getParam("max_inspection_target_width", this->maxTargetWidth_)){
			this->descendHgt_ = 5.0;
			cout << "[AutoFlight]: No max width for inspection. Use default: 5.0m" << endl;
		}
		else{
			
			cout << "[AutoFlight]: Max width for inspection: " << this->maxTargetWidth_ << "m" << endl;
		}

		// descend height
		if (not this->nh_.getParam("inspection_descend_height", this->descendHgt_)){
			this->descendHgt_ = 3.0;
			cout << "[AutoFlight]: No descend height for inspection. Use default: 0.3m" << endl;
		}
		else{
			cout << "[AutoFlight]: Descend height for inspection: " << this->descendHgt_ << "m" << endl;
		}
	}

	void inspector::initPlanner(){
		this->pwlPlanner_ = new trajPlanner::pwlTraj (this->nh_);
		this->rrtPlanner_ = new globalPlanner::rrtStarOctomap<3> (this->nh_);
	}

	void inspector::run(){
		this->lookAround();

		this->forward();

		this->lookAround(); // check the wall condition

		bool targetReach = this->hasReachTarget();
		cout << "reach target: " << targetReach << endl;

		this->moveUp();

		this->lookAround();


		// check surroundings and dimensions of the surface
		this->checkSurroundings();

		// inspect the surface by zig-zag path
		// this->inspect();
	
		// go back to the start position
		this->backward();
	}


	void inspector::lookAround(){
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
		this->pwlPlanner_->updatePath(lookAroundPath, true); // true: use yaw angle in the message

		double t = 0.0;
		ros::Rate r (1.0/this->sampleTime_);
		ros::Time tStart = ros::Time::now();
		while (ros::ok() and (not this->isReach(ps) or t < this->pwlPlanner_->getDuration())){
			ros::Time tCurr = ros::Time::now();
			t = (tCurr - tStart).toSec();
			geometry_msgs::PoseStamped targetPose = this->pwlPlanner_->getPose(t);
			this->updateTarget(targetPose);
			r.sleep();
		}
	}


	void inspector::forward(){
		nav_msgs::Path forwardPath = this->getForwardPath();
		this->pwlPlanner_->updatePath(forwardPath);

		double t = 0.0;
		ros::Rate r (1.0/this->sampleTime_);
		ros::Time tStart = ros::Time::now();
		geometry_msgs::PoseStamped pGoal = forwardPath.poses.back();
		while (ros::ok() and not this->isReach(pGoal)){
			ros::Time tCurr = ros::Time::now();
			t = (tCurr - tStart).toSec();
			geometry_msgs::PoseStamped ps = this->pwlPlanner_->getPose(t);
			this->updateTarget(ps);
			r.sleep();
		}
	}

	void inspector::moveUp(){
		nav_msgs::Path upwardPath;
		geometry_msgs::PoseStamped pCurr;
		geometry_msgs::PoseStamped pHgt;
		pCurr.pose = this->odom_.pose.pose;
		pHgt = pCurr;
		pHgt.pose.position.z = this->maxTargetHgt_;
		std::vector<geometry_msgs::PoseStamped> upwardPathVec {pCurr, pHgt};
		upwardPath.poses = upwardPathVec;
		this->pwlPlanner_->updatePath(upwardPath);

		double t = 0.0;
		ros::Rate r (1.0/this->sampleTime_);
		ros::Time tStart = ros::Time::now();
		geometry_msgs::PoseStamped pGoal = upwardPath.poses.back();
		while (ros::ok() and not this->isReach(pGoal)){
			ros::Time tCurr = ros::Time::now();
			t = (tCurr - tStart).toSec();
			geometry_msgs::PoseStamped ps = this->pwlPlanner_->getPose(t);
			this->updateTarget(ps);
			r.sleep();
		}

	}

	void inspector::checkSurroundings(){

	}

	void inspector::inspect(){
		nav_msgs::Path zigZagPath = this->generateZigZagPath();
		this->pwlPlanner_->updatePath(zigZagPath, true);
		
		double t = 0.0;
		ros::Rate r (1.0/this->sampleTime_);
		ros::Time tStart = ros::Time::now();
		geometry_msgs::PoseStamped pGoal = zigZagPath.poses.back();
		while (ros::ok() and not this->isReach(pGoal)){
			ros::Time tCurr = ros::Time::now();
			t = (tCurr - tStart).toSec();
			geometry_msgs::PoseStamped ps = this->pwlPlanner_->getPose(t);
			this->updateTarget(ps);
			r.sleep();
		}	
	}

	void inspector::backward(){
		nav_msgs::Path backPath;
		std::vector<double> startVec = this->getVecPos();
		std::vector<double> goalVec {0.0, 0.0, this->takeoffHgt_};
		this->rrtPlanner_->updateStart(startVec);
		this->rrtPlanner_->updateGoal(goalVec);
		this->rrtPlanner_->makePlan(backPath);
		// modify back path to make it rotate at the start location
		this->pwlPlanner_->updatePath(backPath);
		this->pwlPlanner_->adjustHeading(this->odom_.pose.pose.orientation);


		double t = 0.0;
		ros::Rate r (1.0/this->sampleTime_);
		ros::Time tStart = ros::Time::now();
		geometry_msgs::PoseStamped pGoal = backPath.poses.back();
		while (ros::ok() and not this->isReach(pGoal)){
			ros::Time tCurr = ros::Time::now();
			t = (tCurr - tStart).toSec();
			geometry_msgs::PoseStamped ps = this->pwlPlanner_->getPose(t);
			this->updateTarget(ps);
			r.sleep();
		}
	}

	bool inspector::hasReachTarget(){
		std::vector<double> range;
		double area = this->findTargetRange(range);
		cout << "Area is: " << area << endl; 
		cout << "x: " << range[0] << " " << range[1] << endl;
		cout << "y: " << range[2] << " " << range[3] << endl;
		cout << "z: " << range[4] << " " << range[5] << endl;

 		bool hasReachTarget;
		if (area >= this->minTargetArea_){
			hasReachTarget = true;
		}
		else{
			hasReachTarget = false;
		}

		this->updateTargetVis(range, hasReachTarget);
		return hasReachTarget;
	}

	void inspector::mapCB(const octomap_msgs::Octomap &msg){
    	octomap::OcTree* treePtr = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(msg));
   	 	this->map_ = std::shared_ptr<octomap::OcTree>(treePtr);
   	 	this->mapRes_ = this->map_->getResolution();
   	 	this->setSurroundingFree(this->getPoint3dPos());
   	 	// cout << "here" << endl;
	}

	visualization_msgs::Marker inspector::getLineMarker(double x1, double y1, double z1, 
													    double x2, double y2, double z2,
													    int id, bool hasReachTarget){
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
		lineMarker.scale.x = 0.1;
		lineMarker.scale.y = 0.1;
		lineMarker.scale.z = 0.1;
		lineMarker.color.a = 1.0; // Don't forget to set the alpha!
		if (hasReachTarget){
			lineMarker.color.r = 0.0;
			lineMarker.color.g = 1.0;
		}
		else{
			lineMarker.color.r = 1.0;
			lineMarker.color.g = 0.0;
		}
		lineMarker.color.b = 0.0;
		return lineMarker;
	}


	void inspector::updateTargetVis(const std::vector<double>& range, bool hasReachTarget){
		double xmin, xmax, ymin, ymax, zmin, zmax;
		xmin = range[0]; xmax = range[1];
		ymin = range[2]; ymax = range[3];
		zmin = range[4]; zmax = range[5];
		
		// 12 line
		std::vector<visualization_msgs::Marker> visVec;
		visualization_msgs::Marker l1 = this->getLineMarker(xmin, ymin, zmin, xmin, ymax, zmin, 1, hasReachTarget);
		visualization_msgs::Marker l2 = this->getLineMarker(xmin, ymax, zmin, xmax, ymax, zmin, 2, hasReachTarget);
		visualization_msgs::Marker l3 = this->getLineMarker(xmax, ymax, zmin, xmax, ymin, zmin, 3, hasReachTarget);
		visualization_msgs::Marker l4 = this->getLineMarker(xmin, ymin, zmin, xmax, ymin, zmin, 4, hasReachTarget);
		visualization_msgs::Marker l5 = this->getLineMarker(xmin, ymin, zmin, xmin, ymin, zmax, 5, hasReachTarget);
		visualization_msgs::Marker l6 = this->getLineMarker(xmin, ymax, zmin, xmin, ymax, zmax, 6, hasReachTarget);
		visualization_msgs::Marker l7 = this->getLineMarker(xmax, ymax, zmin, xmax, ymax, zmax, 7, hasReachTarget);
		visualization_msgs::Marker l8 = this->getLineMarker(xmax, ymin, zmin, xmax, ymin, zmax, 8, hasReachTarget);
		visualization_msgs::Marker l9 = this->getLineMarker(xmin, ymin, zmax, xmin, ymax, zmax, 9, hasReachTarget);
		visualization_msgs::Marker l10 = this->getLineMarker(xmin, ymax, zmax, xmax, ymax, zmax, 10, hasReachTarget);
		visualization_msgs::Marker l11 = this->getLineMarker(xmax, ymax, zmax, xmax, ymin, zmax, 11, hasReachTarget);
		visualization_msgs::Marker l12 = this->getLineMarker(xmin, ymin, zmax, xmax, ymin, zmax, 12, hasReachTarget);


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

		this->targetVisVec_ = visVec;
	}

	void inspector::publishTargetVis(){
		ros::Rate r (1);
		while (ros::ok()){
			this->targetVisMsg_.markers = this->targetVisVec_;
			this->targetVisPub_.publish(this->targetVisMsg_);
			r.sleep();
		}
	}

	geometry_msgs::PoseStamped inspector::getForwardGoal(){
		octomap::point3d p = this->getPoint3dPos();
	
		// cast a ray along the x direction and check collision
		float res = this->mapRes_;
		octomap::point3d pForward = p;
		while (ros::ok() and not this->checkCollision(pForward)){
			pForward.x() += res;
			// cout << res << endl;
			// cout << pForward << endl;
		}
		octomap::point3d pGoal = pForward;
		pGoal.x() -= res;
		pGoal.x() -= this->safeDist_;
		if (pGoal.x() <= p.x()){ // we don't let the robot move backward somehow
			pGoal.x() = p.x(); // at least stay at the same position
		}

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

	std::vector<double> inspector::getVecPos(){
		double x, y, z;
		x = this->odom_.pose.pose.position.x;
		y = this->odom_.pose.pose.position.y;
		z = this->odom_.pose.pose.position.z;
		std::vector<double> vecPos {x, y, z};
		return vecPos;
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


	double inspector::findTargetRangeAxis(const octomap::point3d& pStart, const octomap::point3d& direction, std::vector<octomap::point3d>& resultVec){
		resultVec.clear();
		bool stopCriteria = false;
		int count = 0;
		while (ros::ok() and not stopCriteria){
			float coeff = count * this->mapRes_;
			octomap::point3d delta = direction * coeff;
			octomap::point3d pCheck = pStart + delta;
			bool hasCollision = this->checkCollisionPoint(pCheck, true);
			if (hasCollision){
				resultVec.push_back(pCheck);
			}
			else{
				stopCriteria = true;
			}
			++count;
		}
		octomap::point3d pLast = resultVec[resultVec.size()-1];
		double dist = 0.0;
		if (direction.x() == 1){
			dist = pLast.x() + this->mapRes_;
		}
		else if (direction.x() == -1){
			dist = pLast.x() - this->mapRes_;
		}
		else if (direction.y() == 1){
			dist = pLast.y() + this->mapRes_;
		}
		else if (direction.y() == -1){
			dist = pLast.y() - this->mapRes_;
		}
		else if (direction.z() == 1){
			dist = pLast.z() + this->mapRes_;
			if (dist <= 0){
				dist = 0;
			}
		}
		else if (direction.z() == -1){
			dist = pLast.z() - this->mapRes_;
			if (dist <= 0){
				dist = 0;
			}
		}
		else{
			cout << "[AutoFlight]: Invalid Direction!" << endl;
		}

		return dist;
	}

	double inspector::findTargetRange(std::vector<double>& range){ // return the area of the target range
		// 1. find the collision point at current location
		octomap::point3d pCurr = this->getPoint3dPos();
		octomap::point3d xPlusDirection (1.0, 0, 0); // along the x axis
		octomap::point3d pCollision;
		bool hitTarget = this->map_->castRay(pCurr, xPlusDirection, pCollision);
		if (not hitTarget){
			return 0.0; // this means the front is unknown
		}
		else{
			// instead of only using the collision point, we check all occupied voxels after collision point
			std::vector<octomap::point3d> pCheckVec;
			this->findTargetRangeAxis(pCollision, xPlusDirection, pCheckVec);
	

			// go through y and z direction of all points in checklist and obtain the maximum range
			octomap::point3d yPlusDirection (0, 1.0, 0);
			octomap::point3d yMinusDirection = -yPlusDirection;
			octomap::point3d zPlusDirection (0, 0, 1.0);
			octomap::point3d zMinusDirection = -zPlusDirection;

			std::vector<double> yminVec, ymaxVec, zminVec, zmaxVec;
			for (octomap::point3d pCheck : pCheckVec){
				std::vector<octomap::point3d> resultVec; // dummy vairable
				double yminTemp = this->findTargetRangeAxis(pCheck, yMinusDirection, resultVec);
				double ymaxTemp = this->findTargetRangeAxis(pCheck, yPlusDirection, resultVec);
				double zminTemp = this->findTargetRangeAxis(pCheck, zMinusDirection, resultVec);
				double zmaxTemp = this->findTargetRangeAxis(pCheck, zPlusDirection, resultVec);
				yminVec.push_back(yminTemp);
				ymaxVec.push_back(ymaxTemp);
				zminVec.push_back(zminTemp);
				zmaxVec.push_back(zmaxTemp);
			}
			double xmin = pCheckVec[0].x() - this->mapRes_;
			double xmax = pCheckVec[pCheckVec.size()-1].x() + this->mapRes_;
			double ymin = *(std::min_element(yminVec.begin(), yminVec.end()));
			double ymax = *(std::max_element(ymaxVec.begin(), ymaxVec.end()));
			double zmin = *(std::min_element(zminVec.begin(), zminVec.end()));
			double zmax = *(std::max_element(zmaxVec.begin(), zmaxVec.end())); 

			range.clear();
			range.push_back(xmin);
			range.push_back(xmax);
			range.push_back(ymin);
			range.push_back(ymax);
			range.push_back(zmin);
			range.push_back(zmax);

			double crossSectionArea = (zmax - zmin) * (ymax - ymin);
			return crossSectionArea;
		}
	}

	octomap::point3d inspector::findInspectionStartPoint(){
		octomap::point3d pCurr = this->getPoint3dPos();

		// check along +y axis
		bool hasCollision = false;
		octomap::point3d pCheck;
		int count = 0;
		while (ros::ok() and not hasCollision){
			pCheck = pCurr;
			pCheck.y() += count * this->mapRes_;
			hasCollision = this->checkCollision(pCheck);
			++count;
		}
		octomap::point3d pChecklimit = pCheck;
		pChecklimit.y() -= this->mapRes_;
		pChecklimit.y() -= this->safeDist_; 

		return pChecklimit;
	}

	std::vector<octomap::point3d> inspector::getInspectionLimit(const octomap::point3d& p){
		std::vector<octomap::point3d> limitVec;

		// Plus Y direction
		octomap::point3d pCheckPlus = p;
		bool hasCollisionPlus = false;
		int countPlus = 0;
		while (ros::ok() and not hasCollisionPlus){
			pCheckPlus.y() += countPlus * this->mapRes_;
			hasCollisionPlus = this->checkCollision(pCheckPlus);
			++countPlus;
		}
		octomap::point3d pLimitPlus = pCheckPlus;
		pLimitPlus.y() -= this->mapRes_;
		pLimitPlus.y() -= this->safeDist_;

		// Minus Y direction
		octomap::point3d pCheckMinus = p;
		bool hasCollisionMinus = false;
		int countMinus = 0;
		while (ros::ok() and not hasCollisionMinus){
			pCheckMinus.y() -= countMinus * this->mapRes_;
			hasCollisionMinus = this->checkCollision(pCheckMinus);
			++countMinus;
		}
		octomap::point3d pLimitMinus = pCheckMinus;
		pLimitMinus.y() += this->mapRes_;
		pLimitMinus.y() += this->safeDist_;

		// check the distance between two limits and the closer one gets first in the vec
		double distPlus = p.distance(pLimitPlus);
		double distMinus = p.distance(pLimitMinus);
		if (distPlus >= distMinus){
			limitVec.push_back(pLimitMinus);
			limitVec.push_back(pLimitPlus);
		}
		else{
			limitVec.push_back(pLimitPlus);
			limitVec.push_back(pLimitMinus);
		}
		// cout << "P Limit Plus: " << pLimitPlus << endl;
		// cout << "p Limit Minus: " << pLimitMinus << endl;
		return limitVec;
	}

	nav_msgs::Path inspector::generateZigZagPath(){
		octomap::point3d pCurr = this->getPoint3dPos();
		octomap::point3d pInspection = this->findInspectionStartPoint();
		nav_msgs::Path zzPath;
		std::vector<geometry_msgs::PoseStamped> zzPathVec;

		// add current pose and first inspection pose to the path
		geometry_msgs::PoseStamped psCurr = this->pointToPose(pCurr);
		geometry_msgs::PoseStamped psInspection = this->pointToPose(pInspection);
		zzPathVec.push_back(psCurr);
		zzPathVec.push_back(psInspection);

		octomap::point3d pInspectionHgt = pInspection;
		double height = pInspectionHgt.z();
		while (ros::ok() and not (height <= this->takeoffHgt_)){
			std::vector<octomap::point3d> pLimitCheck = this->getInspectionLimit(pInspectionHgt);
			geometry_msgs::PoseStamped psLimit1 = this->pointToPose(pLimitCheck[0]);
			geometry_msgs::PoseStamped psLimit2 = this->pointToPose(pLimitCheck[1]);
			zzPathVec.push_back(psLimit1);
			zzPathVec.push_back(psLimit2);
			pInspectionHgt = pLimitCheck[1];
			pInspectionHgt.z() -= this->descendHgt_;
			geometry_msgs::PoseStamped psLimit2Lower = this->pointToPose(pInspectionHgt);
			zzPathVec.push_back(psLimit2Lower); 
			height = pInspectionHgt.z();
		}
		
		zzPath.poses = zzPathVec;
		return zzPath;
	}

	geometry_msgs::PoseStamped inspector::pointToPose(const octomap::point3d& p){
		geometry_msgs::PoseStamped ps;
		ps.pose = this->odom_.pose.pose;
		ps.pose.position.x = p.x();
		ps.pose.position.y = p.y();
		ps.pose.position.z = p.z();
		return ps;
	}
}