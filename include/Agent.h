#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

class Agent{

	public:
		Agent();
		~Agent();
		ros::Publisher RobotNode_stage_pub
		ros::Subscriber StageOdo_Sub
	
	private:
		geometry_msgs::Twist RobotNode_cmdvel;
}


