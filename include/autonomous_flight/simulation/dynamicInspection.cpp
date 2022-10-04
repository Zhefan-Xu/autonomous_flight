/*
	FILE: dynamicInspection.cpp
	-----------------------------
	Implementation of dynamic inspection
*/

#include <autonomous_flight/simulation/dynamicInspection.h>

namespace AutoFlight{
	dynamicInspection::dynamicInspection(const ros::NodeHandle& nh) : flightBase(nh){
		this->initModules();
	}

	void dynamicInspection::initParam(){
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

	void dynamicInspection::registerCallback(){
		// planner callback
		this->plannerTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dynamicInspection::plannerCB, this);

		// check wall callback
		this->checkWallTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dynamicInspection::checkWallCB, this);
	}

	void dynamicInspection::run(){
		this->takeoff();

		this->registerCallback();
	}

	void dynamicInspection::plannerCB(const ros::TimerEvent&){
		if (this->flightState_ == FLIGHT_STATE::NAVIGATE){
			// navigate to the goal position

			// first try with pwl trajectory if not work try with the global path planner
			
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
			double minX = endPos(0);
			double maxX = endPos(0) + maxThickness;
			double minY = endPos(1);
			double maxY = endPos(0);
			double minZ = endPos(2);
			double maxZ = endPos(2);			

			const double searchResolution = 0.3;
			for (double xs=endPos(0); xs<endPos(0)+maxThickness; xs+=searchResolution){
				Eigen::Vector3d p (xs, endPos(1), endPos(2));
				Eigen::Vector3d pYP, pYN, pZP, pZN;
				const double maxRayLengthWall = 7.0;
				
				// cast ray in +y direction
				bool ypSuccess = this->map_->castRay(p, Eigen::Vector3d (0, 1, 0), pYP, maxRayLengthWall);

				// cast ray in -y direction
				bool ynSuccess = this->map_->castRay(p, Eigen::Vector3d (0, -1, 0), pYN, maxRayLengthWall);

				// cast ray in +z direction
				bool zpSuccess = this->map_->castRay(p, Eigen::Vector3d (0, 0, 1), pZP, maxRayLengthWall);

				// cast ray in -y direction
				bool znSuccess = this->map_->castRay(p, Eigen::Vector3d (0, 0, -1), pZN, maxRayLengthWall);
			
				// update range
				if (pYN(1) < minY){
					minY = pYN(1);
				}

				if (pYP(1) > maxY){
					maxY = pYP(1);
				}

				if (pYN(1) < minY){
					minY = pYN(1);
				}

				if (pZP(2) > maxZ){
					maxZ = pZP(2);
				}

				if (pZN(2) < minZ){
					minZ = pZN(2);
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

	void dynamicInspection::changeGoal(double x, double y, double z){
		geometry_msgs::PoseStamped ps;
		ps.pose.position.x = x;
		ps.pose.position.y = y;
		ps.pose.position.z = z;
		this->changeGoal(ps);
	}

	void dynamicInspection::changeGoal(const geometry_msgs::PoseStamped& goal){
		this->goal_ = goal;
	}

	void dynamicInspection::changeState(const FLIGHT_STATE& flightState){
		this->flightState_ = flightState;
	}

	void dynamicInspection::updateWallRange(const std::vector<double>& wallRange){
		this->wallRange_ = wallRange;
	}
}