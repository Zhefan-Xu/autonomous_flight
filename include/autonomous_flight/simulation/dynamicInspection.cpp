/*
	FILE: dynamicInspection.cpp
	-----------------------------
	Implementation of dynamic inspection
*/

#include <autonomous_flight/simulation/dynamicInspection.h>

namespace AutoFlight{
	dynamicInspection::dynamicInspection(const ros::NodeHandle& nh) : flightBase(nh){

	}

	void dynamicInspection::registerCallback(){
		// planner callback
		this->plannerTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dynamicInspection::plannerCB, this);
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
}