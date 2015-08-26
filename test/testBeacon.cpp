// Bring in necessary test files
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
// Include google test
#include <gtest/gtest.h>
#include "Beacon.h"

double x = 0;
double y = 0;
bool messageReceive = false;
ros::Publisher robotNode_location_pub;
Beacon beacon1;

TEST(Beacon, constructBeacon) {
	Beacon beacon;
	EXPECT_EQ(beacon.getX(), 0);
	EXPECT_EQ(beacon.getY(), 0);
	EXPECT_EQ(beacon.getTheta(), 0);
	EXPECT_EQ(beacon.getLin(), 0);
	EXPECT_EQ(beacon.getAng(), 0);

};

TEST(Beacon, receivePosition) {
    
    ros::Rate loop_rate(10); 
    nav_msgs::Odometry tempMessage;
    tempMessage.pose.pose.position.x = -32;
	tempMessage.pose.pose.position.y = 18.62;
	robotNode_location_pub.publish(tempMessage);
    while(!messageReceive) {
        loop_rate.sleep();
        ros::spinOnce();
    } 
    EXPECT_EQ(beacon1.getX(),-32);
    EXPECT_EQ(beacon1.getY(),18.62);
};

void stage_callback(nav_msgs::Odometry msg) {
    messageReceive = true;
	beacon1.stageOdom_callback(msg);
}

int main(int argc, char**argv) {
	ros::init(argc,argv,"testBeacon");
    ros::NodeHandle n;
    robotNode_location_pub = n.advertise<nav_msgs::Odometry>("/beacon1/",1000);
    ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>("/beacon1/",1000,stage_callback);
     
	testing::InitGoogleTest(&argc, argv);
     
	return RUN_ALL_TESTS();
}
