// Bring in necessary test files
#include "GardenWorker.h"

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

int main(int argc, char**argv) {
	ros::init(argc, argv, "testGardenWorker");
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}