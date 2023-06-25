/*
	FILE: dynamicExploration.h
	-----------------------------
	header of dynamic exploration
*/
#ifndef DYNAMIC_EXPLORATION
#define DYNAMIC_EXPLORATION
#include <autonomous_flight/px4/flightBase.h>


namespace AutoFlight{
	class dynamicExploration : flightBase{
	private:
		ros::NodeHandle nh_;
	
	public:
		dynamicExploration();
		dynamicExploration(const ros::NodeHandle& nh);

		void run();
	};
}
#endif