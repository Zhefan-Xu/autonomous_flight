/*
	FILE: orca_node.cpp
	Execution file for orca: obstacle avoidance
*/

#include <ros/ros.h>
#include <autonomous_flight/simulation/quadcopter_command.h>
#include <reactive_planner/ORCAPlanner.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "orca_node");
	ros::NodeHandle nh;

	AutoFlight::quadCommand qm (nh);
	qm.takeoff();
	std::vector<double> goal = qm.getGoal();
	
	reactivePlanner::orcaPlanner op (nh);
	op.updateGoal(std::vector<double> {goal[0], goal[1]});

	ros::Rate r (50);
	while (ros::ok()){
		if (op.isReach() == true){
			std::vector<double> goal = qm.getGoal();
			op.updateGoal(std::vector<double> {goal[0], goal[1]});
		}


		std::vector<double> outputVel;
		op.makePlan(outputVel, true);
		qm.setVelocity(outputVel[0], outputVel[1], 0);
		cout << "Vel: " << outputVel[0] <<  " " << outputVel[1] << endl;
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}