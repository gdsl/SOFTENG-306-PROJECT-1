// Bring in necessary test files
#include "AlphaPerson.h"

// Include google test
#include <gtest/gtest.h>

/**
 * Unit tests for Person Robot - tests basic methods e.g. constructor
 */

/**
 * Test default alphaPerson constructor
 */
TEST(AlphaPerson, constructAlphaPerson) {
	AlphaPerson alphaPerson;
	EXPECT_EQ(alphaPerson.getX(), 0);
	EXPECT_EQ(alphaPerson.getY(), 0);
	EXPECT_EQ(alphaPerson.getTheta(), 0);
	EXPECT_EQ(alphaPerson.getLin(), 0);
	EXPECT_EQ(alphaPerson.getAng(), 0);
	EXPECT_FALSE(alphaPerson.getDesireLocation());
	EXPECT_EQ(alphaPerson.getMinDistance(), 30.0);
	EXPECT_EQ(alphaPerson.getObstacleAngle(), 270);
};

int main(int argc, char**argv) {
	ros::init(argc, argv, "testAlphaPerson");
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
