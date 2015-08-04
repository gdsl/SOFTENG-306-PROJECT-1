#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>

class Robot{

	public:
		Robot();
		~Robot();
		ros::Publisher robotNode_stage_pub;
		ros::Subscriber stageOdo_Sub;
		
		void stageOdom_callback(nav_msgs::Odometry msg);
		void moveTo(geometry_msgs::Point point);
		void updateOdometry();
	private:
		geometry_msgs::Twist robotNode_cmdvel;
};


