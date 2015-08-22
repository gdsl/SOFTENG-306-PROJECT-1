#ifndef SE306PROJECT_SRC_Cat_H
#define SE306PROJECT_SRC_Cat_H
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include "Animal.h"

/**
 * Cat class header file
 * This class is for the declaration of method
 */
class Cat: public Animal {
public:
	Cat();
    Cat(double x, double y);
	virtual ~Cat();
	
};

#endif /* SE306PROJECT_SRC_ALPHA_CAT_ */



