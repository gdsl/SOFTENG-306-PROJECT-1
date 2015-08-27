// Bring in necessary test files
#include "GardenWorker.h"
#include "PickerRobot.h"
#include "se306project/weed_status.h"

// Include google test
#include <gtest/gtest.h>

/**
 * Test default alphaPerson constructor
 */
TEST(GardenWorker, constructGardenWorker) {
	GardenWorker gardenWorker;
	EXPECT_EQ(gardenWorker.getX(), 0);
	EXPECT_EQ(gardenWorker.getY(), 0);
	EXPECT_EQ(gardenWorker.getTheta(), 0);
	EXPECT_EQ(gardenWorker.getLin(), 0);
	EXPECT_EQ(gardenWorker.getAng(), 0);
	EXPECT_FALSE(gardenWorker.getDesireLocation());
	EXPECT_EQ(gardenWorker.getMinDistance(), 30.0);
	EXPECT_EQ(gardenWorker.getObstacleAngle(), 270);
};

TEST(GardenWorker, testCommunicationPartnerSetter) {
	GardenWorker gardenWorker;
	gardenWorker.setCommunicationPartners(5);
	EXPECT_EQ(gardenWorker.getCommunicationPartners(), 5);
}

TEST(GardenWorker, testGardenWorkerFSM) {
	GardenWorker gardenWorker;
	EXPECT_EQ(gardenWorker.getStatus(), "Idle");
	gardenWorker.next("Communicate");
	EXPECT_EQ(gardenWorker.getStatus(), "Communicating");
	gardenWorker.next("Stop");
	EXPECT_EQ(gardenWorker.getStatus(), "Idle");
	gardenWorker.next("Move");
	EXPECT_EQ(gardenWorker.getStatus(), "Moving");
	gardenWorker.next("Pull");
	EXPECT_EQ(gardenWorker.getStatus(), "Pull Weed");
	gardenWorker.next("Finish");
	EXPECT_EQ(gardenWorker.getStatus(), "Done");
	gardenWorker.next("Stop");
	EXPECT_EQ(gardenWorker.getStatus(), "Idle");
}

TEST(GardenWorker, testRemovalRequest) {
	ros::NodeHandle n;

	// initialise robot nodes
	GardenWorker gardenWorker;
	PickerRobot pickerRobot;

	pickerRobot.weed_obstacle_pub = n.advertise<se306project::weed_status>("/weed/",1000);

	// create default size
	gardenWorker.tallweed_pose_sub = new ros::Subscriber[1];

	gardenWorker.tallweed_pose_sub[0] = n.subscribe<se306project::weed_status>("/weed/",1000,&GardenWorker::weedRemovalRequest, &gardenWorker);
	gardenWorker.gardenworker_weedinfo_pub = n.advertise<se306project::gardenworker_status>("weedinfo",1000);

	se306project::weed_status msg;
	msg.pos_x = 10;
	msg.pos_y = 10;
	
	pickerRobot.weed_obstacle_pub.publish(msg);

	//wait a bit to make sure message has been received.
	ros::Rate loop_rate(10);
	while(gardenWorker.getMessagesSent() == 0) {
		ros::spinOnce();
		loop_rate.sleep();
	}

	// check that gardenworker has received message
	EXPECT_EQ(gardenWorker.getTargetX(), 10);
	EXPECT_EQ(gardenWorker.getTargetY(), 10);
	// there are zero communication partners
	EXPECT_EQ(gardenWorker.getStatus(), "Moving");
}

int main(int argc, char**argv) {
	ros::init(argc, argv, "testGardenWorker");
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}