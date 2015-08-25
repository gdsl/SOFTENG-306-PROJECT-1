#include "Person.h"

/**
 * Header file for Neighbour. 
 */
class Neighbour: public Person
{
public:
	Neighbour();
	Neighbour(double x,double y);
	Neighbour(double x, double y, double theta, double linearVelocity, double angularVelocity);

	ros::Publisher Neighbour_status_pub;

	// Finds nearest Robot and updates variable
	void updateNearestRobot(nav_msgs::Odometry msg);
	// Update status
	void next(std::string action);
	// override stagelaser callback
	void stageLaser_callback(sensor_msgs::LaserScan msg);
	// override stageodom callback
	void stageOdom_callback(nav_msgs::Odometry msg);

	// getter methods
	int getTargetX();
	int getTargetY();
private:
	int targetX;
	int targetY;
};
