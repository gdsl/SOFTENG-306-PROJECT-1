#include "Person.h"
#include "Constants.h"

Person::Person():Entity() {

}

Person::Person(double x, double y):Entity(x,y,0,0,0,HUMAN_ANGLE_NOT_PROCESS) {

}

Person::~Person() {

}
