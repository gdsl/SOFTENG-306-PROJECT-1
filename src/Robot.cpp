#include "Robot.h"
#include <nav_msgs/Odometry.h>

Robot::Robot():Entity(0,0,0,0,0) {
	// TODO Auto-generated constructor stub

}

Robot::~Robot() {
	// TODO Auto-generated destructor stub
}

void Robot::StageLaser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number
	

}
