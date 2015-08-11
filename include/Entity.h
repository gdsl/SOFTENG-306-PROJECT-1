/**
 * Entity Header file.
 * Here is where we declare method specification for Entity.
 */
#ifndef SE306PROJECT_SRC_ENTITY_H_
#define SE306PROJECT_SRC_ENTITY_H_
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "Movement.h"
#include <vector>
#include <string>

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
		void stageOdom_callback(nav_msgs::Odometry msg);

                void stageLaser_callback(sensor_msgs::LaserScan msg);
		void atLocation();
		// Movement methods
		void move();
		void movementComplete();
		void addMovement(std::string type,double amount,double velocity);
		void moveForward(double distance,double vel, std::string direction);
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
		double getMinDistance();
		double getObstacleAngle();
		bool getDesireLocation();

		//movement queue
        std::vector<Movement> movementQueue;

	private:
		//positions
		double x;
		double y;
		double theta;

		// velocity
		double linearVelocity;
		double angularVelocity;

		double minDistance;
		double obstacleAngle;
		
		//direction robot facing
		enum Direction {WEST, SOUTH, EAST, NORTH};
		Direction directionFacing=WEST;//initialse to west originally
		//boolean for if the robot is at desire location
		bool desireLocation;
		// Expresses velocity in free space broken into its linear and angular parts
		// http://docs.ros.org/jade/api/nav_msgs/html/msg/Odometry.html
		geometry_msgs::Twist robotNode_cmdvel;
};
#endif
