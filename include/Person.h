#ifndef SE306PROJECT_SRC_PERSON_H
#define SE306PROJECT_SRC_PERSON_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "Entity.h"

class Person : public Entity {

public:
    Person();
    virtual ~Person();
};

#endif
