#include <gtest/gtest.h>
#include "CarrierRobot.h"
#include "Robot.h"

/**
 * Unit tests for Carrier Robot - tests basic methods e.g. constructor
 */

/*
 * Test for Entity constructor. Tests that default values are assigned correctly.
 */
TEST(CarrierRobot, constructCarrier) {
	CarrierRobot carrierRobot = CarrierRobot();
	EXPECT_EQ(carrierRobot.getX(), 0);
	EXPECT_EQ(carrierRobot.getY(), 0);
	EXPECT_EQ(carrierRobot.getTheta(), 0);
	EXPECT_EQ(carrierRobot.getLin(), 0);
	EXPECT_EQ(carrierRobot.getAng(), 0);
	EXPECT_FALSE(carrierRobot.getDesireLocation());
}

/*
 * Test for CarrierRobot constructor with parameters. Tests that parameter values are assigned correctly.
 */
TEST(CarrierRobot, constructWithArgsCarrier) {
	CarrierRobot carrierRobot = CarrierRobot(10, 10, 5, 2, 3, "Idle");
	EXPECT_EQ(carrierRobot.getX(), 10);
	EXPECT_EQ(carrierRobot.getY(), 10);
	EXPECT_EQ(carrierRobot.getTheta(), 5);
	EXPECT_EQ(carrierRobot.getLin(), 2);
	EXPECT_EQ(carrierRobot.getAng(), 3);
    EXPECT_EQ(carrierRobot.getStatus(), "Idle");
    EXPECT_EQ(carrierRobot.getState(), Robot::IDLE);
	EXPECT_FALSE(carrierRobot.getDesireLocation());
}

int main(int argc,char **argv) {
	ros::init(argc, argv, "testCarrierRobot");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
