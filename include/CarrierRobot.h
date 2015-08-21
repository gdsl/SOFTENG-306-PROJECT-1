#ifndef SE306PROJECT_SRC_CARRIERROBOT_H_
#define SE306PROJECT_SRC_CARRIERROBOT_H_
#include "Robot.h"
#include "ros/ros.h"

/**
 * CarrierRobot Header file.
 * Here is where we declare method specification.
 */
class CarrierRobot: public Robot {
public:
	CarrierRobot();
    CarrierRobot(double x,double y,double theta,double linearVel, double angularVel,std::string status);
	CarrierRobot(std::string status);
	virtual ~CarrierRobot();
	virtual void stateLogic();
};

#endif /* SE306PROJECT_SRC_CARRIERROBOT_H_ */
