#ifndef SE306PROJECT_SRC_BLINDPERSON_H_
#define SE306PROJECT_SRC_BLINDPERSON_H_
#include "Person.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

class BlindPerson: public Person
{
public:
	BlindPerson();
	BlindPerson(double x, double y);
	BlindPerson(double x, double y, double theta, double linearVelocity, double angularVelocity);
	virtual ~BlindPerson();

private:

};

#endif /* SE306PROJECT_SRC_BLINDPERSON_H_ */
