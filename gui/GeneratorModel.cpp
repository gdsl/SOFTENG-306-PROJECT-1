#include "GeneratorModel.h"

/**
 * Default constructor for GeneratorModel.
 * Initializes all variables and sets it to zero. Some have fixed value
 */
GeneratorModel::GeneratorModel()
{
	pickerRobots = 0;
	carrierRobots = 0;
	// fixed for now
	rowLength = 70;
	rowWidth = 0.0;
	poleTrunkSpacing = 0.0;
	rowCount = 0;
	gardeners = 0;
	dogs = 0;
	cats = 0;
	workers = 0;
	blindPerson = 0;
	neighbours = 0;
	tractors = 1;
	beacons = 0;
	weed = 10;
}

int GeneratorModel::getTotalNodes() {
	return weed + beacons + pickerRobots + carrierRobots + workers + dogs + cats + blindPerson + neighbours + gardeners;
}
