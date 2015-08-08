#ifndef SE306PROJECT_SRC_ROBOT_H_
#define SE306PROJECT_SRC_ROBOT_H_
#include "Entity.h"
#include <nav_msgs/Odometry.h>

class Robot :public Entity {
public:
	Robot();
	virtual ~Robot();
};

#endif /* SE306PROJECT_SRC_ROBOT_H_ */
