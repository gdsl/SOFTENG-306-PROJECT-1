#include <gtest/gtest.h>
#include "PickerRobot.h"

//function for example
int add(int a, int b) {
    return a+b;
}

TEST(Example,testcase1) {
    EXPECT_EQ(5,add(2,3));
}

TEST(PickerRobot, constructPicker) {
	PickerRobot pickerRobot;
	EXPECT_EQ(pickerRobot.getLin(), 0);
}

int main(int argc,char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
