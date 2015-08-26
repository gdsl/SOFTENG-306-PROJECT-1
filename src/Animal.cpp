#include "Animal.h"
#include "Constants.h"

Animal::Animal():Entity() {

}

Animal::Animal(double x, double y):Entity(x,y,0,0,0,ANIMAL_ANGLE_NOT_PROCESS) {

}

Animal::~Animal() {

}
