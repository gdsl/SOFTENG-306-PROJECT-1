#include "Person.h"
#include "se306project/gardenworker_status.h"
#include "se306project/weed_status.h"
#include "se306project/robot_status.h"

/**
 * Header file for GardenWorker. GardenWorker role is to remove weeds placed around the orchard.
 */
class GardenWorker: public Person
{
public:
	GardenWorker();
	GardenWorker(double x, double y, double theta, double linearVelocity, double angularVelocity);

	ros::Publisher gardenworker_status_pub;
	ros::Publisher gardenworker_weedinfo_pub;
	ros::Subscriber *tallweed_pose_sub;
	ros::Subscriber *gardenworker_status_sub;

	// Finds nearest weed and updates variable
	//void updateNearestWeed(nav_msgs::Odometry msg);
	void weedRemovalRequest(const se306project::weed_status msg);
	void weedRemovalDelegation(const se306project::gardenworker_status msg);

	// Update status
	void next(std::string action);
	// override stagelaser callback
	void stageLaser_callback(sensor_msgs::LaserScan msg);
	// override stageodom callback
	void stageOdom_callback(nav_msgs::Odometry msg);

	// getter methods
	double getTargetX();
	double getTargetY();
	double getInitialX();
	double getInitialY();
	int getCommunicationPartners();
	int getMessagesSent();

	// setter methods
	void setCommunicationPartners(int communicationPartners);
	void setInitialX(double initialX);
	void setInitialY(double initialY);
private:
	double targetX;
	double targetY;
	double initialX;
	double initialY;
	int communicationPartners;
	int messagesSent;
	int messagesReceived;
	bool closestToWeed;
	time_t c_start;
	time_t c_end;
	bool pulled;
};
