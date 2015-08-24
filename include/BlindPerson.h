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
    void followDog();
    
private:
    
};
