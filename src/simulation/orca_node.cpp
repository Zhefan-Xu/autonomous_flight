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

		if (std::abs(outputVel[0]) >= 100){ // debug
			cout << "============================debug========================" << endl;
			// Robot Position and Goal position
			nav_msgs::Odometry robotOdom = op.getOdom();
			cout << "Current Position: " << robotOdom.pose.pose.position.x << ", " << robotOdom.pose.pose.position.y << endl;
			cout << "GOal Position: " << goal[0] << ", " << goal[1] << endl;


			// prefered vel in world frame
			Eigen::Vector2d velPref = op.getVelPreferVec();
			cout << "prefered velocity: " << velPref[0] << ", " << velPref[1] << endl;

			// each obstacle position and information
			std::vector<onboard_vision::Obstacle> obstacleVec;
			onboard_vision::ObstacleList obMsg= op.getObstacles();
			obstacleVec = obMsg.obstacles;
			int count = 0;
			for (onboard_vision::Obstacle ob : obstacleVec){
				cout << "Obstacle " << count << ": position: " << "(" << ob.px  << ", " << ob.py << "). Vel: " << "(" << ob.vx << ", " << ob.vy << ")." << endl; 
				cout << "size: " << std::max(ob.xsize, ob.ysize) << endl;
				++count;  
			}


			break; 
		}

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}