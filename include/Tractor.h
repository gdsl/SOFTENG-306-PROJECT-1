#ifndef SE306PROJECT_SRC_TRACTOR_H_
#define SE306PROJECT_SRC_TRACTOR_H_
#include "ros/ros.h"
#include "Entity.h"
#include <nav_msgs/Odometry.h>

/**
 * CarrierRobot Header file.
 * Here is where we declare method specification.
 */
class Tractor :public Entity {
public:
	Tractor();
    Tractor(double x,double y,double theta,double linearVel, double angularVel);
	virtual ~Tractor();
};

#endif /* SE306PROJECT_SRC_TRACTOR_H_ */
