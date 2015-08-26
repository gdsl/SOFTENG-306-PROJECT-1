#ifndef SE306PROJECT_SRC_ANIMAL_H
#define SE306PROJECT_SRC_ANIMAL_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "Entity.h"

class Animal : public Entity {

public:
	Animal();
	Animal(double x, double y);
	virtual ~Animal();
};

#endif
