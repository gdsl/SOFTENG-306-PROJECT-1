#include "Robot.h"
#include <nav_msgs/Odometry.h>

Robot::Robot():Entity(0,0,0,0,0) {

}

Robot::Robot(double x,double y,double theta,double linearVel, double angularVel) : Entity(x, y, theta, linearVel, angularVel) {

}

Robot::~Robot() {

}

Robot::State Robot::getState() {
	return state;
}

void Robot::setState(Robot::State state) {
    this->state = state;   
}
