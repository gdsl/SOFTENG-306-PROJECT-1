/**
 * Model which represents the object that is passed into the Generator to create world file
 */

class GeneratorModel
{
public:
	GeneratorModel();

	int pickerRobots;
	int carrierRobots;
	int rowLength;
	float rowWidth;
	float poleTrunkSpacing;
	int rowCount;
	int gardeners;
	int dogs;
	int cats;
	int workers;
	int blindPerson;
	int neighbors;
	int tractors;
	int beacons;
	int weed;
	
	int getTotalNodes();
};
