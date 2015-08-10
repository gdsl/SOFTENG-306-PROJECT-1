/**
 * Movement header file.
 * Here is where we declare method specification for movement.
 * Created on: 10/08/2015
 * Author: Guyver Fu
 */
#ifndef SE306PROJECT_SRC_MOVEMENT_H_
#define SE306PROJECT_SRC_MOVEMENT_H_
#include <string>

class Movement {
public:
	Movement();
	Movement(std::string type,double amount,double velocity);
	virtual ~Movement();
	std::string getType();
	double getPos();
	double getVel();
	void setType(std::string MovementType);
	void setPos(double pos);
	void setVel(double velocity);
private:
	std::string MovementType;//variable for type of movement (forward_x, forward_y or rotation)
	double pos; //Variable for abs pos of the movement destination
	double velocity; //varaible for velocity of movement
};

#endif /* SE306PROJECT_SRC_MOVEMENT_H_ */
