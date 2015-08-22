#ifndef SE306PROJECT_SRC_TALLWEED_H
#define SE306PROJECT_SRC_TALLWEED_H
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include "Entity.h"

/**
 * Beacon class header file
 * This class is for the declaration of method
 */
class TallWeed: public Entity {
public:
	TallWeed();
	virtual ~TallWeed();
	
};

#endif /* SE306PROJECT_SRC_TALLWEED_ */




