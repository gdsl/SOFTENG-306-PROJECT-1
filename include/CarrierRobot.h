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

    //getMethod
    bool isInitialMovement();
    bool isCarrierInFront();
    double getYDistanceTravel();
    double getXDistanceTravel();
    
    //setMethod
    void setYDistanceTravel(double y);
    void setXDistanceTravel(double x);
    void setCarrierInFront(bool front);
    void setInitialMovement(bool initial);

private:
    bool carrierInFront;
    bool initialMovement;
    double yDistanceTravel;
    double xDistanceTravel;
};



#endif /* SE306PROJECT_SRC_CARRIERROBOT_H_ */
