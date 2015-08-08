#ifndef SE306PROJECT_SRC_PICKERROBOT_H
#define SE306PROJECT_SRC_PICKERROBOT_H
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include "Robot.h"
/**
 * PickerRobot class
 * This class is for the implementation of the picker robot
 */

class PickerRobot: public Robot {
public:
	PickerRobot();
	virtual ~PickerRobot();
};

#endif /* SE306PROJECT_SRC_PICKERROBOT_H_ */




