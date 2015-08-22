#include "se306project/gardenworker_status.h"

/**
 * Header file for GardenWorker. GardenWorker role is to remove weeds placed around the orchard.
 */
class GardenWorker: public Person
{
public:
	GardenWorker();
	GardenWorker(double x, double y, double theta, double linearVelocity, double angularVelocity);

	ros::Publisher gardenworker_status_pub;
	ros::Subscriber tallweed_pose_sub;

	// Finds nearest weed and updates variable
	void updateNearestWeed(TallWeed weed);
	void increment();
	// Update status
	void next(std::string action);
private:
	TallWeed nearestWeed;
	uint weedCounter;
};
