#include <gtest/gtest.h>
#include "PickerRobot.h"
#include "CarrierRobot.h"

//function for example
int add(int a, int b) {
    return a+b;
}

TEST(Example,testcase1) {
    EXPECT_EQ(5,add(2,3));
}

/*
 * Test for Entity constructor. Tests that default values are assigned correctly.
 */
TEST(PickerRobot, constructPicker) {
	PickerRobot pickerRobot = PickerRobot();
	EXPECT_EQ(pickerRobot.getX(), 0);
	EXPECT_EQ(pickerRobot.getY(), 0);
	EXPECT_EQ(pickerRobot.getTheta(), 0);
	EXPECT_EQ(pickerRobot.getLin(), 0);
	EXPECT_EQ(pickerRobot.getAng(), 0);
	EXPECT_FALSE(pickerRobot.getDesireLocation());
}

/*
 * Test for Entity setPose() method.
 * Checks to see if the parameters given are assigned to object fields correctly.
 */
TEST(PickerRobot, setPoseOfPicker) {
	PickerRobot pickerRobot = PickerRobot();
	pickerRobot.setPose(5, 10, 3);
	EXPECT_EQ(pickerRobot.getX(), 5);
	EXPECT_EQ(pickerRobot.getY(), 10);
	EXPECT_EQ(pickerRobot.getTheta(), 3);
}

/*
 * Test for Entity setVelocity() method.
 * Checks to see if the parameters given are assigned to object fields correctly.
 */
TEST(PickerRobot, setVelocityOfPicker) {
	PickerRobot pickerRobot = PickerRobot();
	pickerRobot.setVelocity(1.5, 0.1);
	EXPECT_EQ(pickerRobot.getLin(), 1.5);
	EXPECT_EQ(pickerRobot.getAng(), 0.1);
}

/*
 * Test for Entity setDesireLocation() method.
 * Checks to see if the parameter given is assigned to object field correctly.
 */
TEST(PickerRobot, setDesireLocationOfPicker) {
	PickerRobot pickerRobot = PickerRobot();
	pickerRobot.setDesireLocation(true);
	EXPECT_TRUE(pickerRobot.getDesireLocation());
}

/*
 * Test for Entity moveForward() method when Robot is NOT near its destination position.
 * Velocity of Robot should be set to value given as the Robot should be moving towards its 
 * horizontal or vertical destination.
 * WILL NOT WORK UNTIL ROBOT PUBLISHERS ARE INITIALIZED IN CONSTRUCTORS.*/
TEST(PickerRobot, moveForwardWhenNotAtDestination) {
	PickerRobot pickerRobot = PickerRobot();
	ros::NodeHandle n;
	pickerRobot.robotNode_stage_pub=n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	pickerRobot.setPose(10, 10, 5);
	pickerRobot.moveForward(15, 1, "x");
	EXPECT_EQ(pickerRobot.getLin(), 1);
}

/*
 * Test for Carrier robot inherit Entity constructor. Tests that default values are assigned correctly.
 */
TEST(CarrierRobot, constructCarrier) {
	CarrierRobot carrierRobot = CarrierRobot();
    ros::NodeHandle n;
	EXPECT_EQ(carrierRobot.getX(), 0);
	EXPECT_EQ(carrierRobot.getY(), 0);
	EXPECT_EQ(carrierRobot.getTheta(), 0);
	EXPECT_EQ(carrierRobot.getLin(), 0);
	EXPECT_EQ(carrierRobot.getAng(), 0);
	EXPECT_FALSE(carrierRobot.getDesireLocation());
}

int main(int argc,char **argv) {
	ros::init(argc,argv,"testRobot");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
