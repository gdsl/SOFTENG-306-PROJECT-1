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
	AlphaPerson alphaPerson = AlphaPerson();
	EXPECT_EQ(alphaPerson.getX(), 0);
	EXPECT_EQ(alphaPerson.getY(), 0);
	EXPECT_EQ(alphaPerson.getTheta(), 0);
	EXPECT_EQ(alphaPerson.getLin(), 0);
	EXPECT_EQ(alphaPerson.getAng(), 0);
	EXPECT_FALSE(alphaPerson.getDesireLocation());
	EXPECT_EQ(alphaPerson.getMinDistance(), 30.0);
	EXPECT_EQ(alphaPerson.getObstacleAngle(), 270);
};

/**
 * Test alphaPerson constructor with X and Y pose arguments
 */
TEST(AlphaPerson, constructWithPoseAlphaPerson) {
	AlphaPerson alphaPerson = AlphaPerson(10, 20);
	EXPECT_EQ(alphaPerson.getX(), 10);
	EXPECT_EQ(alphaPerson.getY(), 20);
	EXPECT_EQ(alphaPerson.getTheta(), 0);
	EXPECT_EQ(alphaPerson.getLin(), 0);
	EXPECT_EQ(alphaPerson.getAng(), 0);
	EXPECT_FALSE(alphaPerson.getDesireLocation());
	EXPECT_EQ(alphaPerson.getMinDistance(), 30.0);
	EXPECT_EQ(alphaPerson.getObstacleAngle(), 270);
};

/*
 * Test for AlphaPerson's setState() method using a State argument specific to AlphaPerson
 */
TEST(AlphaPerson, setStateOfAlphaPerson) {
	AlphaPerson alphaPerson = AlphaPerson(10, 20);
    alphaPerson.setState(AlphaPerson::TRIMMING);
    EXPECT_EQ(alphaPerson.getState(), AlphaPerson::TRIMMING);
}

int main(int argc, char**argv) {
	ros::init(argc, argv, "testAlphaPerson");
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
