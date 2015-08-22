#ifndef SE306PROJECT_SRC_ROBOT_H_
#define SE306PROJECT_SRC_ROBOT_H_
#include "Entity.h"
#include <nav_msgs/Odometry.h>

/**
 * Robot Header file.
 * Here is where we declare method specification for Robot.
 */
class Robot :public Entity {
public:
	Robot();
	Robot(double x,double y,double theta,double linearVel, double angularVel);
	virtual ~Robot();
	//method for the robots state logic for transition and implementation
	virtual void stateLogic()=0;
    //State enum for all subclasses of Robot
    enum State {IDLE, DISPATCH, GO_TO_NEXT_BEACON, PICKING, FULL_BIN, FINISHED ,TRANSPORTING, QUEUE ,ARRIVED,MOVING};
    State getState();
    void setState(State state);

private:
    State state;
};

#endif /* SE306PROJECT_SRC_ROBOT_H_ */
