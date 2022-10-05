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

		// minimum wall area
		if (not this->nh_.getParam("min_wall_area", this->minWallArea_)){
			this->minWallArea_ = 20;
			cout << "[AutoFlight]: No minimum wall area param. Use default 20m^2." << endl;
		}
		else{
			cout << "[AutoFlight]: Minimum wall area is set to: " << this->minWallArea_ << "m^2." << endl;
		}		
	}

	void dynamicInspection::initModules(){
		// initialize map
		this->map_.reset(new mapManager::occMap (this->nh_));
		// this->map_.reset(new mapManager::dynamicMap ());
		// this->map_->initMap(this->nh_);

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

		// piecewise linear trajectory publisher
		this->pwlTrajPub_ = this->nh_.advertise<nav_msgs::Path>("inspection/pwl_trajectory", 10);

		// bspline trajectory publisher
		this->bsplineTrajPub_ = this->nh_.advertise<nav_msgs::Path>("inspection/bspline_trajectory", 10);

		// wall visualization publisher
		this->wallVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("inspection/wall", 10);
	}

	void dynamicInspection::registerCallback(){
		// planner callback
		this->plannerTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dynamicInspection::plannerCB, this);

		// trajectory execution callback
		this->trajExeTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dynamicInspection::trajExeCB, this);

		// check wall callback
		this->checkWallTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dynamicInspection::checkWallCB, this);

		// visualization callback
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.05), &dynamicInspection::visCB, this);
	}

	void dynamicInspection::run(){
		this->takeoff();

		this->registerCallback();
	}

	void dynamicInspection::plannerCB(const ros::TimerEvent&){
		if (this->flightState_ == FLIGHT_STATE::FORWARD){
			// navigate to the goal position

			// first try with pwl trajectory if not work try with the global path planner
			// todo: add replan condition

			bool replan = (this->td_.needReplan(1.0/3.0));
			if (replan){
				nav_msgs::Path simplePath;
				geometry_msgs::PoseStamped pStart, pGoal;
				pStart.pose = this->odom_.pose.pose;
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
					}
				}
			}
			// if multiple times cannot navigate to the goal, change the state to EXPLORE

			return;
		}

		if (this->flightState_ == FLIGHT_STATE::EXPLORE){
			// generate new local exploration goal

			// change the state to NAVIGATE

			return;
		}

		if (this->flightState_ == FLIGHT_STATE::INSPECT){
			// generate zig-zag path and exexute. No replan needed

			return;
		}

		if (this->flightState_ == FLIGHT_STATE::BACK){
			// turn back

			// set new goal to the origin position

			// change the state to NAVIGATE
			return;
		}
	}

	void dynamicInspection::trajExeCB(const ros::TimerEvent&){
		if (not this->td_.init){
			return;
		}
		this->updateTarget(this->td_.getPoseWithoutYaw(this->odom_.pose.pose));
	}

	void dynamicInspection::checkWallCB(const ros::TimerEvent&){
		// use current robot position, check the occcupancy of the predefined wall size bounding box

		// const double maxWallWidth = 10.0; // The maximum width of the wall
		// const double maxWallHeight = 10.0; // the maximum height of the wall
		const double maxThickness = 3.0; // The maximum thickness of the wall

		// 1. find the occupied point in front of the robot
		Eigen::Vector3d currPos (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		Eigen::Vector3d direction (1, 0, 0);
		Eigen::Vector3d endPos;
		double maxRayLength = 5.0;
		bool castRaySuccess = this->map_->castRay(currPos, direction, endPos, maxRayLength);
		
		if (castRaySuccess){
			// search for the wall region
			double minX = endPos(0) - this->map_->getRes();
			double maxX = endPos(0) + maxThickness;
			double minY = endPos(1);
			double maxY = endPos(1);
			double minZ = endPos(2);
			double maxZ = endPos(2);			

			const double searchResolution = 0.3;
			for (double xs=endPos(0); xs<endPos(0)+maxThickness; xs+=searchResolution){
				Eigen::Vector3d p (xs, endPos(1), endPos(2));
				Eigen::Vector3d pYP, pYN, pZP, pZN;
				const double maxRayLengthWall = 7.0;
				
				// cast ray in +y direction
				bool ypSuccess = this->castRayOccupied(p, Eigen::Vector3d (0, 1, 0), pYP, maxRayLengthWall);

				// cast ray in -y direction
				bool ynSuccess = this->castRayOccupied(p, Eigen::Vector3d (0, -1, 0), pYN, maxRayLengthWall);

				// cast ray in +z direction
				bool zpSuccess = this->castRayOccupied(p, Eigen::Vector3d (0, 0, 1), pZP, maxRayLengthWall);

				// cast ray in -z direction
				bool znSuccess = this->castRayOccupied(p, Eigen::Vector3d (0, 0, -1), pZN, maxRayLengthWall);

				// update range
				if (pYP(1) > maxY){
					maxY = pYP(1);
				}

				if (pYN(1) < minY){
					minY = pYN(1);
				}

				
				if (pZP(2) > maxZ){
					maxZ = pZP(2);
				}

				if (not znSuccess){
					minZ = 0.0;
				}
				else{
					if (pZN(2) < minZ){
						minZ = pZN(2);
					}
				}



				bool noWall = (not ypSuccess) and (not ynSuccess) and (not zpSuccess) and (not znSuccess);
				if (noWall){
					maxX = xs;
					break;
				}
			}

			
			std::vector<double> wallRange {minX, maxX, minY, maxY, minZ, maxZ};
			this->updateWallRange(wallRange);
		}
		else{
			std::vector<double> wallRange {0, 0, 0, 0, 0, 0};	
			this->updateWallRange(wallRange);
		}
	}

	void dynamicInspection::visCB(const ros::TimerEvent&){
		this->goalPub_.publish(this->goal_);
		if (this->pwlTrajMsg_.poses.size() != 0){
			this->pwlTrajPub_.publish(this->pwlTrajMsg_);
		}
		if (this->bsplineTrajMsg_.poses.size() != 0){
			this->bsplineTrajPub_.publish(this->bsplineTrajMsg_);
		}

		if (this->wallRange_.size() != 0){
			this->getWallVisMsg(this->wallVisMsg_);
			this->wallVisPub_.publish(this->wallVisMsg_);
		}

	}

	geometry_msgs::PoseStamped dynamicInspection::getForwardGoal(){
		// if not meet wall, the goal will always be aggresive (+10 meter in front of the robot)

		geometry_msgs::PoseStamped goal;
		goal.pose = this->odom_.pose.pose;
		goal.pose.position.x += 10.0;
		// goal.pose.position.x = 1.0;
		this->goal_ = goal;
		return goal;
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
		double area = (this->wallRange_[3] - this->wallRange_[2]) * (this->wallRange_[5] - this->wallRange_[4]);
		if (area > this->minWallArea_){
			return true;
		}
		else{
			return false;
		}
	}

	void dynamicInspection::updateWallRange(const std::vector<double>& wallRange){
		this->wallRange_ = wallRange;
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

	void dynamicInspection::getWallVisMsg(visualization_msgs::MarkerArray& msg){
		msg.markers.clear();
		double xmin, xmax, ymin, ymax, zmin, zmax;
		xmin = this->wallRange_[0]; xmax = this->wallRange_[1];
		ymin = this->wallRange_[2]; ymax = this->wallRange_[3];
		zmin = this->wallRange_[4]; zmax = this->wallRange_[5];


		bool isWall = this->isWallDetected();
		std::vector<visualization_msgs::Marker> visVec;
		visualization_msgs::Marker l1 = this->getLineMarker(xmin, ymin, zmin, xmin, ymax, zmin, 1, isWall);
		visualization_msgs::Marker l2 = this->getLineMarker(xmin, ymax, zmin, xmax, ymax, zmin, 2, isWall);
		visualization_msgs::Marker l3 = this->getLineMarker(xmax, ymax, zmin, xmax, ymin, zmin, 3, isWall);
		visualization_msgs::Marker l4 = this->getLineMarker(xmin, ymin, zmin, xmax, ymin, zmin, 4, isWall);
		visualization_msgs::Marker l5 = this->getLineMarker(xmin, ymin, zmin, xmin, ymin, zmax, 5, isWall);
		visualization_msgs::Marker l6 = this->getLineMarker(xmin, ymax, zmin, xmin, ymax, zmax, 6, isWall);
		visualization_msgs::Marker l7 = this->getLineMarker(xmax, ymax, zmin, xmax, ymax, zmax, 7, isWall);
		visualization_msgs::Marker l8 = this->getLineMarker(xmax, ymin, zmin, xmax, ymin, zmax, 8, isWall);
		visualization_msgs::Marker l9 = this->getLineMarker(xmin, ymin, zmax, xmin, ymax, zmax, 9, isWall);
		visualization_msgs::Marker l10 = this->getLineMarker(xmin, ymax, zmax, xmax, ymax, zmax, 10, isWall);
		visualization_msgs::Marker l11 = this->getLineMarker(xmax, ymax, zmax, xmax, ymin, zmax, 11, isWall);
		visualization_msgs::Marker l12 = this->getLineMarker(xmin, ymin, zmax, xmax, ymin, zmax, 12, isWall);


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
}