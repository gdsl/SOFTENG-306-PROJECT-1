#ifndef SE306PROJECT_SRC_ENTITYMOCK_H_
#define SE306PROJECT_SRC_ENTITYMOCK_H_
#include "Entity.h"
#include <nav_msgs/Odometry.h>

/**
 * EntityMock Header file.
 * Here is where we declare method specification for EntityMock.
 */
class EntityMock :public Entity {
public:
	EntityMock();
	EntityMock(double x,double y,double theta,double linearVel, double angularVel);
	virtual ~EntityMock();
	//method for the robots state logic for transition and implementation
    //State enum for all subclasses of EntityMock
    enum State {IDLE, DISPATCH, GO_TO_NEXT_BEACON, PICKING, FULL_BIN, FINISHED ,TRANSPORTING, QUEUE ,ARRIVED,MOVING,SERVICED};
    State getState();
    void setState(State state);

private:
    State state;
};

#endif /* SE306PROJECT_SRC_ENTITYMOCK_H_ */
