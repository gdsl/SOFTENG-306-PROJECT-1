// Bring in necessary test files
#include "AlphaDog.h"

// Include google test
#include <gtest/gtest.h>

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

/**
 * Test AlphaDog setPose() method.
 */
TEST(AlphaDog, setPoseOfDog) {
	AlphaDog alphaDog;
	alphaDog.setPose(20, 10, 30);
	EXPECT_EQ(alphaDog.getX(), 20);
	EXPECT_EQ(alphaDog.getY(), 10);
	EXPECT_EQ(alphaDog.getTheta(), 30);
}

/*
 * Test for Entity setVelocity() method.
 * Checks to see if the parameters given are assigned to object fields correctly.
 */
TEST(AlphaDog, setVelocityOfDog) {
	AlphaDog alphaDog;
	alphaDog.setVelocity(2, 3);
	EXPECT_EQ(alphaDog.getLin(), 2);
	EXPECT_EQ(alphaDog.getAng(), 3);
}

/*
 * Test for Entity setDesireLocation() method.
 * Checks to see if the parameter given is assigned to object field correctly.
 */
TEST(AlphaDog, setDesireLocationOfDog) {
	AlphaDog alphaDog;
	alphaDog.setDesireLocation(true);
	EXPECT_TRUE(alphaDog.getDesireLocation());
	alphaDog.setDesireLocation(false);
	EXPECT_FALSE(alphaDog.getDesireLocation());
}

/*
 * Test for Entity moveForward() method when Robot is NOT near its destination position.
 * Velocity of Robot should be set to value given as the Robot should be moving towards its 
 * horizontal or vertical destination.
 * WILL NOT WORK UNTIL ROBOT PUBLISHERS ARE INITIALIZED IN CONSTRUCTORS.*/
//TEST(AlphaDog, moveForwardWhenNotAtDestination) {
//	AlphaDog alphaDog;
//	ros::NodeHandle n;
//	                   alphaDog.robotNode_stage_pub=n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
//	alphaDog.setPose(10, 10, 5);
//	alphaDog.moveForward(15, 1, "x");
//	EXPECT_EQ(alphaDog.getLin(), 1);
//}

int main(int argc, char**argv) {
	ros::init(argc,argv,"testAlphaDog");
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
