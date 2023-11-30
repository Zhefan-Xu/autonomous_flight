/*
	FILE: localPlanner.h
	---------------------------------------------
	local planner function implementation
*/

#include <autonomous_flight/simulation/localPlanner.h>

namespace AutoFlight{

	localPlanner::localPlanner(const ros::NodeHandle& nh, double desiredVel, double desiredAcc) : nh_(nh), 
		desiredVel_(desiredVel), desiredAcc_(desiredAcc){}

	void localPlanner::loadOccMap(const std::shared_ptr<mapManager::occMap> occMap){
		this->occMap_ = occMap;
		// initialize polynomial trajectory planner
		this->polyTraj_.reset(new trajPlanner::polyTrajOccMap (this->nh_));
		this->polyTraj_->setMap(this->occMap_);
		this->polyTraj_->updateDesiredVel(this->desiredVel_);
		this->polyTraj_->updateDesiredAcc(this->desiredAcc_);

		// initialize piecewise linear trajectory planner
		this->pwlTraj_.reset(new trajPlanner::pwlTraj (this->nh_));

		// initialize bspline trajectory planner
		this->bsplineTraj_.reset(new trajPlanner::bsplineTraj (this->nh_));
		this->bsplineTraj_->setMap(this->occMap_);
		this->bsplineTraj_->updateMaxVel(this->desiredVel_);
		this->bsplineTraj_->updateMaxAcc(this->desiredAcc_);

		// initialize the time optimizer
		this->timeOptimizer_.reset(new timeOptimizer::bsplineTimeOptimizer (this->nh_));
		this->timeOptimizer_->setMap(this->occMap_);	
	}

	void localPlanner::loadDynamicMap(const std::shared_ptr<mapManager::dynamicMap> dynamicMap){
		this->dynamicMap_ = dynamicMap;

		// initialize polynomial trajectory planner
		this->polyTraj_.reset(new trajPlanner::polyTrajOccMap (this->nh_));
		this->polyTraj_->setMap(this->dynamicMap_);
		this->polyTraj_->updateDesiredVel(this->desiredVel_);
		this->polyTraj_->updateDesiredAcc(this->desiredAcc_);

		// initialize piecewise linear trajectory planner
		this->pwlTraj_.reset(new trajPlanner::pwlTraj (this->nh_));

		// initialize bspline trajectory planner
		this->bsplineTraj_.reset(new trajPlanner::bsplineTraj (this->nh_));
		this->bsplineTraj_->setMap(this->dynamicMap_);
		this->bsplineTraj_->updateMaxVel(this->desiredVel_);
		this->bsplineTraj_->updateMaxAcc(this->desiredAcc_);

		// initialize the time optimizer
		this->timeOptimizer_.reset(new timeOptimizer::bsplineTimeOptimizer (this->nh_));
		this->timeOptimizer_->setMap(this->dynamicMap_);
	}
}