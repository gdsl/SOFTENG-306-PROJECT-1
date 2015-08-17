#ifndef SE306PROJECT_SRC_ALPHA_PERSON_H
#define SE306PROJECT_SRC_ALPHA_PERSON_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "Person.h"

class AlphaPerson : public Person {

public:
    AlphaPerson();
    AlphaPerson(double x, double y);
    virtual ~AlphaPerson();
};

#endif
