// Bring in necessary test files
#include "Neighbour.h"

// Include google test
#include <gtest/gtest.h>

/**
 * Unit tests for Neighbour Robot - tests basic methods e.g. constructor
 */

/**
 * Test default Neighbour constructor
 */
TEST(Neighbour, constructNeighbour) {
	Neighbour neighbour;
	EXPECT_EQ(neighbour.getX(), 0);
	EXPECT_EQ(neighbour.getY(), 0);
	EXPECT_EQ(neighbour.getTheta(), 0);
	EXPECT_EQ(neighbour.getLin(), 0);
	EXPECT_EQ(neighbour.getAng(), 0);
	EXPECT_FALSE(neighbour.getDesireLocation());
	EXPECT_EQ(neighbour.getMinDistance(), 30.0);
	EXPECT_EQ(neighbour.getObstacleAngle(), 270);
};

TEST(Neighbour, testSetOriginX) {
	Neighbour neighbour;
	neighbour.setOriginX(100);
	EXPECT_EQ(neighbour.getOriginX(), 100);
}

TEST(Neighbour, testSetOriginY) {
	Neighbour neighbour;
	neighbour.setOriginY(50);
	EXPECT_EQ(neighbour.getOriginY(), 50);
}

int main(int argc, char**argv) {
	ros::init(argc, argv, "testNeighbour");
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
