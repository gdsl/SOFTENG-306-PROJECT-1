#ifndef SE306PROJECT_SRC_ALPHADOG_H
#define SE306PROJECT_SRC_AlPHADOG_H
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include "Animal.h"

/**
 * AlphaDog class header file
 * This class is for the declaration of method
 */
class AlphaDog: public Animal {
public:
	AlphaDog();
    AlphaDog(double x, double y);
    enum State {TOP, BOTTOM, RIGHT, LEFT};
	virtual ~AlphaDog();
	
};

#endif /* SE306PROJECT_SRC_ALPHA_DOG_ */




