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
		Entity(double x, double y, double theta, double linearVelocity, double angularVelocity);
		// update the position of the robot
		void setPose(int x, int y, double theta);

		// update the velocity of the robot
		void setVelocity(double linearVelocity, double angularVelocity);

		// Publisher and Subscriber. Public available to update.
		ros::Publisher robotNode_stage_pub;
		ros::Subscriber stageOdo_Sub;
		ros::Subscriber baseScan_Sub;

		// Callback methods
		void atLocation();
		void stageOdom_callback(nav_msgs::Odometry msg);
		void stageLaser_callback(sensor_msgs::LaserScan msg);
		void moveForward(double distance,double vel);
		void rotate(double angleToRotateTo,double angleSpeed);
		void faceNorth(double angleSpeed);
		void faceSouth(double angleSpeed);
		void faceEast(double angleSpeed);
		void faceWest(double angleSpeed);
		void updateOdometry();
		void setDesireLocation(bool desireLocation);
		//get method
		double getX();
		double getY();
		double getTheta();
		double getLin();
		double getAng();
		bool getDesireLocation();

	private:
		//positions
		double x;
		double y;
		double theta;

		// velocity
		double linearVelocity;
		double angularVelocity;

		//boolean for if the robot is at desire location
		bool desireLocation;
		// Expresses velocity in free space broken into its linear and angular parts
		// http://docs.ros.org/jade/api/nav_msgs/html/msg/Odometry.html
		geometry_msgs::Twist robotNode_cmdvel;
};
#endif
