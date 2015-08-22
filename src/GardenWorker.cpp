#include "GardenWorker.h"

/**
 * Default constructor for GardenWorker
 */
GardenWorker::GardenWorker():GardenWorker(0, 0, 0, 0, 0) {}

/**
 * Call super class constructor
 */
GardenWorker::GardenWorker(double x, double y, double theta, double linearVelocity, double angularVelocity)
: Entity(x, y, theta, linearVelocity, angularVelocity)
{
	nearestWeed = 0;
	weedCounter = 0;
	status = "Idle";
}

/**
 * Destructor
 */
GardenWorker::~GardenWorker(){}

/**
 * Update the number of weed pulled by garden worker
 */
void GardenWorker::increment()
{
	weedCounter++;
}

/**
 * Update nearest
 */
void GardenWorker::updateNearestWeed(TallWeed weed)
{
	this->nearestWeed = weed;
}

/**
 * Represents FSM for GardenWorker. Given an action, update the current status
 */
void GardenWorker::next(std::string action)
{
	if (status.compare("Idle")==0) {
		if (action.compare("Move")==0) {
			setStatus("Moving");
		} else if (action.compare("Pull")==0) {
			setStatus("Pull Weed");
		}
	} else if (status.compare("Moving")==0) {
		if (action.compare("Pull")==0) {
			setStatus("Pull Weed");
		} else if (action.compare("Stop")==0) {
			setStatus("Idle");
		}
	} else if (status.compare("Pull Weed")==0) {
		if (action.compare("Finish")==0) {
			setStatus("Done");
		}
	} else if (status.compare("Done")==0) {
		setStatus("Idle");
	}
}
