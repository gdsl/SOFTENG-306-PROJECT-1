// Include testing class
#include "Movement.h"

// include google test
#include <gtest/gtest.h>

/**
 * Unit tests for Movement class - checks basic methods e.g. constructor
 */

/**
 * Test default constructor for Movement
 */
TEST(Movement, constructMovementDefault) {
	Movement movement;
	EXPECT_EQ(movement.getType(), "rotation");
	EXPECT_EQ(movement.getPos(), 0);
	EXPECT_EQ(movement.getVel(), 0);
}

/**
 * Test overloaded constructor for Movement
 */
TEST(Movement, constructMovementOverload) {
	Movement movement("forward_x", 10, 2);
	EXPECT_EQ(movement.getType(), "forward_x");
	EXPECT_EQ(movement.getPos(), 10);
	EXPECT_EQ(movement.getVel(), 2);
}

TEST(Movement, setMovementType) {
	Movement movement;
	// Change type
	movement.setType("forward_y");
	EXPECT_EQ(movement.getType(), "forward_y");
}

TEST(Movement, setMovementPos) {
	Movement movement;
	// Change pos
	movement.setPos(1.5);
	EXPECT_EQ(movement.getPos(), 1.5);
}

TEST(Movement, setMovementVel) {
	Movement movement;
	// Change vel
	movement.setVel(3.25);
	EXPECT_EQ(movement.getVel(), 3.25);
}

int main(int argc, char**argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
