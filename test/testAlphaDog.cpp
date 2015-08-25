// Bring in necessary test files
#include "AlphaDog.h"

// Include google test
#include <gtest/gtest.h>

/**
 * Unit tests for Dog Robot - tests basic methods e.g. constructor
 */

/**
 * Test default alphaDog constructor
 */
TEST(AlphaDog, constructAlphaDog) {
	AlphaDog alphaDog;
	EXPECT_EQ(alphaDog.getX(), 0);
	EXPECT_EQ(alphaDog.getY(), 0);
	EXPECT_EQ(alphaDog.getTheta(), 0);
	EXPECT_EQ(alphaDog.getLin(), 0);
	EXPECT_EQ(alphaDog.getAng(), 0);
	EXPECT_FALSE(alphaDog.getDesireLocation());
	EXPECT_EQ(alphaDog.getMinDistance(), 30.0);
	EXPECT_EQ(alphaDog.getObstacleAngle(), 270);
};

int main(int argc, char**argv) {
	ros::init(argc,argv,"testAlphaDog");
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
