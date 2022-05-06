/*
	FILE: quadcopter_command.h
	basic command for quadcopter in simulation
*/

#ifndef QUADCOPTERCOMMAND_H
#define QUADCOPTERCOMMAND_H

namespace AutoFlight{
	class quadCommand{
	private:
		ros::NodeHandle nh_;


	public:
		quadCommand(const ros::NodeHandle& nh);
		void takeoff();
		void land();
		void setVelocity(double vx, double vy, double vz);
		void setPosition(double x, double y, double z, double yaw=0);
		void resetPosition(double x, double y, double z=0, double yaw=0);
	};


	quadCommand::quadCommand(const ros::NodeHandle& nh) : nh_(nh){

	}

	void quadCommand::takeoff(){

	}

	void quadCommand::land(){

	}

	void quadCommand::setVelocity(double vx, double vy, double vz){

	}

	void quadCommand::setPosition(double x, double y, double z, double yaw){
		
	}

	void quadCommand::resetPosition(double x, double y, double z, double yaw){

	}
}
#endif