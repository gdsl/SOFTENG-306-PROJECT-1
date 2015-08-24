#include <gtest/gtest.h>
#include "Entity.h"
#include "EntityMock.h"

/**
 * Unit test for Entity. Tests basic methods such as getters and setters.
 */


/*
 * Test for EntityMock setPose() method.
 * Checks to see if the parameters given are assigned to object fields correctly.
 */
TEST(EntityMock, setPoseOfEntityMock) {
	EntityMock entityMock = EntityMock();
	entityMock.setPose(5, 10, 3);
	EXPECT_EQ(entityMock.getX(), 5);
	EXPECT_EQ(entityMock.getY(), 10);
	EXPECT_EQ(entityMock.getTheta(), 3);
}

/*
 * Test for EntityMock setVelocity() method.
 * Checks to see if the parameters given are assigned to object fields correctly.
 */
TEST(EntityMock, setVelocityOfEntityMock) {
	EntityMock entityMock = EntityMock();
	entityMock.setVelocity(1.5, 0.1);
	EXPECT_EQ(entityMock.getLin(), 1.5);
	EXPECT_EQ(entityMock.getAng(), 0.1);
}

/*
 * Test for EntityMock setDesireLocation() method.
 * Checks to see if the parameter given is assigned to object field correctly.
 */
TEST(EntityMock, setDesireLocationOfEntityMock) {
	EntityMock entityMock = EntityMock();
	entityMock.setDesireLocation(true);
	EXPECT_TRUE(entityMock.getDesireLocation());
}

/*
 * Test for EntityMock getMovementQueueSize() method.
 * Checks to see if the velocity of the robot has changed
 * after adding a movement to the EntityMock's queue and then calling move.
 */
TEST(EntityMock, checkMovementQueueSizeEntityMock) {
	EntityMock entityMock = EntityMock();
	ros::NodeHandle n;
    entityMock.addMovement("forward_x", 5, 1);
    EXPECT_EQ(entityMock.getMovementQueueSize(), 1);
}

/*
 * Test for EntityMock getAvoidanceQueueSize() method.
 * Checks to see if the velocity of the robot has changed
 * after adding a movement to the EntityMock's queue and then calling move.
 */
TEST(EntityMock, checkAvoidanceQueueSizeEntityMock) {
	EntityMock entityMock = EntityMock();
	ros::NodeHandle n;
    entityMock.addMovementFront("forward_x", 5, 1, 1);
    EXPECT_EQ(entityMock.getAvoidanceQueueSize(), 1);
}


/*
 * Test for EntityMock addMovement() method.
 * Checks to see if the velocity of the robot has changed
 * after adding a movement to the EntityMock's queue and then calling move.
 */
TEST(EntityMock, addMovementCheckVelocityEntityMock) {
	EntityMock entityMock = EntityMock();
	ros::NodeHandle n;
	entityMock.robotNode_stage_pub=n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	entityMock.setPose(10, 10, 5);
	entityMock.addMovement("forward_x", 5, 1);
    entityMock.move();
	EXPECT_EQ(entityMock.getLin(), 1);
}


int main(int argc,char **argv) {
	ros::init(argc, argv, "testEntityMock");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
