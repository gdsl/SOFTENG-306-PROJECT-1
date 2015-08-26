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

/*
 * Test for CarrierRobot's setState() method using a State argument specific to Carrier
 */
TEST(CarrierRobot, setStateOfCarrier) {
	CarrierRobot carrierRobot = CarrierRobot(10, 10, 5, 2, 3, "Idle");
    carrierRobot.setState(Robot::TRANSPORTING);
    EXPECT_EQ(carrierRobot.getState(), Robot::TRANSPORTING);
}

//TEST VARIABLES
int msgCount = 0;

///*
// * Method that process the carrier robot message received.
// * This method is called when message is received.
// */
//void receiveCarrierRobotStatus(const se306project::carrier_status::ConstPtr& msg) {
//    msgCount++;
//}
//
///*
// * Test for CarrierRobot's carrier_status message publisher. Checks to see if the
// * custom message is published correctly.
// */
//TEST(CarrierRobot, checkCarrierStatusMsgCarrier) {
//    ros::NodeHandle n; 
//    CarrierRobot carrierRobot = CarrierRobot(10, 10, 5, 2, 3, "Idle");
//    ros::Publisher pub=n.advertise<se306project::carrier_status>("status",1000);
//    ros::Subscriber sub = n.subscribe<se306project::carrier_status>("status",1000, receiveCarrierRobotStatus);
//    
//    //carrier status message initialisation
//	se306project::carrier_status status_msg;
//    
//    status_msg.status=carrierRobot.getStatus();		//add status to message
//    status_msg.pos_x=carrierRobot.getX(); //add x to message to broadcast
//    status_msg.pos_y=carrierRobot.getY();//add y to message to broadcast
//    status_msg.pos_theta=carrierRobot.getTheta(); //add angle to message to broadcast
//    status_msg.obstacle = carrierRobot.getObstacleStatus();
//    pub.publish(status_msg);
//    ros::spinOnce();
//    //only checking to see if msg received at the moment
//    EXPECT_EQ(msgCount, 1);
//    
//}

int main(int argc,char **argv) {
    testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "testCarrierRobot");
    ros::NodeHandle n;    
    return RUN_ALL_TESTS();
}
