#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>

/**
 * Parent class for Robot nodes.
 * Note: Unless the class has a virtual method, it is still a normal class??
 */

class Robot
{
	public:
		// Default constructor
		Robot();

		// Constructor with initialized variables.
		Robot(int x, int y, double theta, double linearVelocity, double angularVelocity);

		// update the position of the robot
		void setPose(int x, int y, double theta);

		// update the velocity of the robot
		void setVelocity(double linearVelocity, double angularVelocity);

		// Publisher and Subscriber. Public available to update.
		ros::Publisher robotNode_stage_pub;
		ros::Subscriber stageOdo_Sub;

		// Callback methods
		void stageOdom_callback(nav_msgs::Odometry msg);
		void moveTo(geometry_msgs::Point point);
		void updateOdometry();

	private:
		// pose
		int x;
		int y;
		double theta;

		// velocity
		double linearVelocity;
		double angularVelocity;

		// Expresses velocity in free space broken into its linear and angular parts
		// http://docs.ros.org/jade/api/nav_msgs/html/msg/Odometry.html
		geometry_msgs::Twist robotNode_cmdvel;
};
