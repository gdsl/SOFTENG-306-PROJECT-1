#include <gtest/gtest.h>
#include "PickerRobot.h"
#include "Robot.h"

/**
 * Unit test for Picker Robot - tests basic methods e.g. constructor
 */

/*
 * Test for PickerRobot default constructor. Tests that default values are assigned correctly.
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
 * Test for PickerRobot constructor with parameters. Tests that parameter values are assigned correctly.
 */
TEST(PickerRobot, constructWithArgsPicker) {
	PickerRobot pickerRobot = PickerRobot(10, 10, 5, 2, 3, "Dispatch", 2);
	EXPECT_EQ(pickerRobot.getX(), 10);
	EXPECT_EQ(pickerRobot.getY(), 10);
	EXPECT_EQ(pickerRobot.getTheta(), 5);
	EXPECT_EQ(pickerRobot.getLin(), 2);
	EXPECT_EQ(pickerRobot.getAng(), 3);
    EXPECT_EQ(pickerRobot.getStatus(), "Dispatch");
    EXPECT_EQ(pickerRobot.getState(), Robot::DISPATCH);
    EXPECT_EQ(pickerRobot.getPickRange(), 2);
	EXPECT_FALSE(pickerRobot.getDesireLocation());
}

/*
 * Test for PickerRobot setPickRange() method.
 * Checks to see if the parameters given are assigned to object fields correctly.
 */
TEST(PickerRobot, setPickRangeOfPicker) {
	PickerRobot pickerRobot = PickerRobot();
	pickerRobot.setPickRange(3);
	EXPECT_EQ(pickerRobot.getPickRange(), 3);
}

/*
 * Test for PickerRobot setBinCapacity() method.
 * Checks to see if the parameters given are assigned to object fields correctly.
 */
TEST(PickerRobot, setBinCapacityOfPicker) {
	PickerRobot pickerRobot = PickerRobot();
	pickerRobot.setBinCapacity(10);
	EXPECT_EQ(pickerRobot.getBinCapacity(), 10);
}

int main(int argc,char **argv) {
	ros::init(argc, argv, "testPickerRobot");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
