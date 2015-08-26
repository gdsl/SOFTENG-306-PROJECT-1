#include "Person.h"
#include <sensor_msgs/LaserScan.h>

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
	// override stageodom callback
	void stageOdom_callback(nav_msgs::Odometry msg);
	
	void setOriginY(double yPos);
	void setOriginX(double xPos);
	double getOriginX();
	double getOriginY();
private:
	
	double originXPos;
	double originYPos;
};
