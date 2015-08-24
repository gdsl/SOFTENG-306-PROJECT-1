#include <gtest/gtest.h>
#include "CarrierRobot.h"

/**
 * Unit tests for Carrier Robot - tests basic methods e.g. constructor
 */

/*
 * Test for Entity constructor. Tests that default values are assigned correctly.
 */
TEST(CarrierRobot, constructPicker) {
	CarrierRobot carrierRobot = CarrierRobot();
	EXPECT_EQ(carrierRobot.getX(), 0);
	EXPECT_EQ(carrierRobot.getY(), 0);
	EXPECT_EQ(carrierRobot.getTheta(), 0);
	EXPECT_EQ(carrierRobot.getLin(), 0);
	EXPECT_EQ(carrierRobot.getAng(), 0);
	EXPECT_FALSE(carrierRobot.getDesireLocation());
}

/*
 * Test for Entity setPose() method.
 * Checks to see if the parameters given are assigned to object fields correctly.
 */
TEST(CarrierRobot, setPoseOfCarrier) {
	CarrierRobot carrierRobot = CarrierRobot();
	carrierRobot.setPose(5, 10, 3);
	EXPECT_EQ(carrierRobot.getX(), 5);
	EXPECT_EQ(carrierRobot.getY(), 10);
	EXPECT_EQ(carrierRobot.getTheta(), 3);
}

/*
 * Test for Entity setVelocity() method.
 * Checks to see if the parameters given are assigned to object fields correctly.
 */
TEST(CarrierRobot, setVelocityOfCarrier) {
	CarrierRobot carrierRobot = CarrierRobot();
	carrierRobot.setVelocity(1.5, 0.1);
	EXPECT_EQ(carrierRobot.getLin(), 1.5);
	EXPECT_EQ(carrierRobot.getAng(), 0.1);
}

/*
 * Test for Entity setDesireLocation() method.
 * Checks to see if the parameter given is assigned to object field correctly.
 */
TEST(CarrierRobot, setDesireLocationOfCarrier) {
	CarrierRobot carrierRobot = CarrierRobot();
	carrierRobot.setDesireLocation(true);
	EXPECT_TRUE(carrierRobot.getDesireLocation());
}

//TEST(MoveForward, moveForwardWhenNotAtDestinationCarrier) {
//	CarrierRobot carrierRobot;
//	ros::NodeHandle n;
//	carrierRobot.robotNode_stage_pub=n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
//	carrierRobot.setPose(10, 10, 5);
//	carrierRobot.moveForward(15, 1, "x");
//	EXPECT_EQ(carrierRobot.getLin(), 1);
//}

int main(int argc,char **argv) {
	ros::init(argc, argv, "testCarrierRobot");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
