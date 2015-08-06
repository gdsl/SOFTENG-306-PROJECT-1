#ifndef SE306PROJECT_SRC_ENTITY_H_
#define SE306PROJECT_SRC_ENTITY_H_
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

/**
 * Parent class for Entity nodes.
 * Note: Unless the class has a virtual method, it is still a normal class??
 */

class Entity
{
	public:
		// Default constructor
		Entity();
		virtual ~Entity();
		// Constructor with initialized variables.
		Entity(int x, int y, double theta, double linearVelocity, double angularVelocity);

		// update the position of the robot
		void setPose(int x, int y, double theta);

		// update the velocity of the robot
		void setVelocity(double linearVelocity, double angularVelocity);

		// Publisher and Subscriber. Public available to update.
		ros::Publisher robotNode_stage_pub;
		ros::Subscriber stageOdo_Sub;

		// Callback methods
		void stageOdom_callback(nav_msgs::Odometry msg);
		void StageLaser_callback(sensor_msgs::LaserScan msg);
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
#endif
