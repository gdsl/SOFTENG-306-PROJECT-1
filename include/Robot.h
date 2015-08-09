#ifndef SE306PROJECT_SRC_ROBOT_H_
#define SE306PROJECT_SRC_ROBOT_H_
#include "Entity.h"
#include <nav_msgs/Odometry.h>

class Robot :public Entity {
public:
	Robot();
	virtual ~Robot();

	// Laser Scanner
	ros::Subscriber baseScan_Sub;

	// Callback methods
	void StageLaser_callback(sensor_msgs::LaserScan msg);
};

#endif /* SE306PROJECT_SRC_ROBOT_H_ */
