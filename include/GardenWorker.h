#include "Person.h"
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
	ros::Subscriber *tallweed_pose_sub;

	// Finds nearest weed and updates variable
	void updateNearestWeed(nav_msgs::Odometry msg);
	void increment();
	// Update status
	void next(std::string action);
	// override stagelaser callback
	void stageLaser_callback(sensor_msgs::LaserScan msg);
	// override stageodom callback
	void stageOdom_callback(nav_msgs::Odometry msg);

	// getter methods
	int getTargetX();
	int getTargetY();
	uint getWeedCounter();
private:
	int targetX;
	int targetY;
	uint weedCounter;
};
