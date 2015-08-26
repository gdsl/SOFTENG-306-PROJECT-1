// Bring in necessary test files
#include "Cat.h"

// Include google test
#include <gtest/gtest.h>

/**
 * Unit tests for Cat Robot - tests basic methods e.g. constructor
 */

/**
 * Test default cat constructor
 */
TEST(Cat, constructCat) {
        Cat cat(0,0);
	EXPECT_EQ(cat.getX(), 0);
	EXPECT_EQ(cat.getY(), 0);
	EXPECT_EQ(cat.getTheta(), 0);
	EXPECT_EQ(cat.getLin(), 0);
	EXPECT_EQ(cat.getAng(), 0);
	EXPECT_FALSE(cat.getDesireLocation());
	EXPECT_EQ(cat.getMinDistance(), 30.0);
	EXPECT_EQ(cat.getObstacleAngle(), 270);
};


int main(int argc, char**argv) {
	ros::init(argc,argv,"testCat");
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
