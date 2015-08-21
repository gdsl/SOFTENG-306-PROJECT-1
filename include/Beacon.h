#ifndef SE306PROJECT_SRC_BEACON_H
#define SE306PROJECT_SRC_BEACON_H
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include "Entity.h"

/**
 * Beacon class header file
 * This class is for the declaration of method
 */
class Beacon: public Entity {
public:
	Beacon();
    Beacon(double x, double y, double theta, double linearVelocity, double angularVelocity);
	virtual ~Beacon();
	
};

#endif /* SE306PROJECT_SRC_BEACON_ */




