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
		Entity(double x, double y, double theta, double linearVelocity, double angularVelocity, int angleNotProcess);
		// update the position of the robot
		void setPose(int x, int y, double theta);

		// update the velocity of the robot
		void setVelocity(double linearVelocity, double angularVelocity);

        //direction robot facing
		enum Direction {WEST, SOUTH, EAST, NORTH};
		//enumeration for avoidance cases
		enum AvoidanceCase{NONE,HALT, LIVING_OBJ,WEED, TREE,FACE_ON, PERPENDICULAR,STATIONARY,ROTATE};

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
		void avoidanceComplete();
		void addMovement(std::string type,double amount,double velocity);
		void addMovementFront(std::string type,double amount,double velocity, int queueType);
		void faceNorth(double angleSpeed);
		void faceSouth(double angleSpeed);
		void faceEast(double angleSpeed);
		void faceWest(double angleSpeed);
		void updateOdometry();
		void setDesireLocation(bool desireLocation);
		void setStatus(std::string status);
        void determineStatus();
		void setLin(double lv);
		void setAng(double av);
		void setObstacleStatus(std::string obstacleStatus);
		void avoidObstacle(double x,double y);
		void flushMovementQueue();
		void moveZ(double pos, double vel, int queueNum);
		bool isRotating();
		//get method
		double getX();
		double getY();
		double getTheta();
		double getLin();
		double getAng();
		double getMinDistance();
		double getObstacleAngle();
		AvoidanceCase getAvoidanceCase();
		bool getDesireLocation();
		std::string getStatus();
		std::string getObstacleStatus();
		int getMovementQueueSize();
		int getAvoidanceQueueSize();
		int getCriticalIntensity();
        Direction getDirectionFacing();

	private:
		//positions
		double x;
		double y;
		double z;
		double theta;

		// velocity
		double linearVelocity;
		double angularVelocity;
		double zVelocity;

		//obstacle avoidance/detection variables
		int criticalIntensity;
		int previousScanIntensity;
		int previousScanNumber;
		int previousScanNumberMin;
		int previousScanNumberMax;
		double minDistance;
		double obstacleAngle;
		int numOfScan;
		double previousScanDistance;
		int angleNotProcess;//variable which store the angle of laser to not process up to eg 30 means only angle from 30 to 150 is processed
		AvoidanceCase avoidanceCase;
		AvoidanceCase previousAvoidanceCase;
		std::string obstacleStatus;
		std::string status;

		//movement queue
        std::vector<Movement> movementQueue;
        std::vector<Movement> avoidanceQueue; //vector for lsit of avoidance movements
		Direction directionFacing=NORTH;//initialse to north originally

		void moveForward(double distance,double vel, std::string direction, int queueNum);
		void rotate(double angleToRotateTo,double angleSpeed,int queueNum);
		//boolean for if the robot is at desire location
		bool desireLocation;
		// Expresses velocity in free space broken into its linear and angular parts
		// http://docs.ros.org/jade/api/nav_msgs/html/msg/Odometry.html
		geometry_msgs::Twist robotNode_cmdvel;
};
#endif

