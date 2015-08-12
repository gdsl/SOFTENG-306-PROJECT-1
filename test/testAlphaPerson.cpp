// Bring in necessary test files
#include "AlphaPerson.h"

// Include google test
#include <gtest/gtest.h>

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
	EXPECT_EQ(alphaPerson.getObstacleDistance(), 270);
};

/**
 * Test AlphaPerson setPose() method.
 */
TEST(AlphaPerson, setPoseOfPerson) {
	AlphaPerson alphaPerson;
	alphaPerson.setPose(10, 30, 40);
	EXPECT_EQ(alphaPerson.getX(), 10);
	EXPECT_EQ(alphaPerson.getY(), 30);
	EXPECT_EQ(alphaPerson.getTheta(), 40);
}

/*
 * Test for Entity setVelocity() method.
 * Checks to see if the parameters given are assigned to object fields correctly.
 */
TEST(AlphaPerson, setVelocityOfPerson) {
	AlphaPerson alphaPerson;
	alphaPerson.setVelocity(4, 1);
	EXPECT_EQ(alphaPerson.getLin(), 4);
	EXPECT_EQ(alphaPerson.getAng(), 1);
}

/*
 * Test for Entity setDesireLocation() method.
 * Checks to see if the parameter given is assigned to object field correctly.
 */
TEST(AlphaPerson, setDesireLocationOfPerson) {
	AlphaPerson alphaPerson;
	alphaPerson.setDesireLocation(true);
	EXPECT_TRUE(alphaPerson.getDesireLocation());
	alphaPerson.setDesireLocation(false);
	EXPECT_FALSE(alphaPerson.getDesireLocation());
}

/*
 * Test for Entity moveForward() method when Robot is NOT near its destination position.
 * Velocity of Robot should be set to value given as the Robot should be moving towards its 
 * horizontal or vertical destination.
 * WILL NOT WORK UNTIL ROBOT PUBLISHERS ARE INITIALIZED IN CONSTRUCTORS.*/
TEST(AlphaPerson, moveForwardWhenNotAtDestination) {
	AlphaPerson alphaPerson;
	ros::NodeHandle n;
	alphaPerson.robotNode_stage_pub=n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	alphaPerson.setPose(10, 10, 5);
	alphaPerson.moveForward(15, 1, "x");
	EXPECT_EQ(alphaPerson.getLin(), 1);
}

int main(int argc, char**argv) {
	ros::init(argc,argv,"testAlphaPerson");
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
