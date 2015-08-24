/**
 * Movement cpp file.
 * Here is where we implementation of movement class is used
 * Created on: 10/08/2015
 * Author: Guyver Fu
 */

#include "Movement.h"

// Default constructor will call
Movement::Movement():Movement("rotation",0,0) {

}

Movement::~Movement() {

}

Movement::Movement(std::string type,double pos,double velocity) {
	// forward_x , forward_y or rotation
	MovementType=type; 
	this->pos=pos;
	this->velocity=velocity;
}

/*
 * Getter method for the type of movement
 */
std::string Movement::getType() {
	return MovementType;
}

/*
 * Getter method for the pos of movement destination
 */
double Movement::getPos() {
	return pos;
}

/*
 * Getter method for the velocity of movement
 */
double Movement::getVel() {
	return velocity;
}

/*
 * Setter method for the type of movement
 */
void Movement::setType(std::string MovementType) {
	this->MovementType= MovementType;
}

/*
 * Setter method for the pos of movement destination
 */
void Movement::setPos(double pos) {
	this->pos=pos;
}

/*
 * Setter method for the velocity of movement
 */
void Movement::setVel(double velocity) {
	this->velocity= velocity;
}
