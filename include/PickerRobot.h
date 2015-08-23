#ifndef SE306PROJECT_SRC_PICKERROBOT_H
#define SE306PROJECT_SRC_PICKERROBOT_H
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include "Robot.h"

/**
 * PickerRobot class header file
 * This class is for the declaration of method
 */
class PickerRobot: public Robot {
public:
	PickerRobot();
	PickerRobot(double x,double y,double theta,double linearVel, double angularVel,std::string status,double pickrange);
	PickerRobot(std::string status);
	virtual ~PickerRobot();
	void movement();
	virtual void stateLogic(ros::NodeHandle n);
	int getBinCapacity();
	void setBinCapacity(int bin_capacity);
	double getPickRange();
	void setPickRange(double pick_range);
    void subscribeNextBeacon(ros::NodeHandle n);

private:
	int bin_capacity;
	double pick_range;
    
};

#endif /* SE306PROJECT_SRC_PICKERROBOT_H_ */




