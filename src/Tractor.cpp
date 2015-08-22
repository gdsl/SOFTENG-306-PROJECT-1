#include <ros/console.h>
#include "std_msgs/String.h"
#include "Tractor.h"

/*
 * Default constructor for carrier Robot
 */
Tractor::Tractor() {
	// TODO Auto-generated constructor stub
}

/*
 * Constructor for carrier Robot with status
 */
Tractor::Tractor(double x,double y,double theta,double linearVel, double angularVel)
	:Entity(x, y, theta, linearVel, angularVel){
}

/*
 * Default destructor for carrier Robot
 */
Tractor::~Tractor() {
	// TODO Auto-generated destructor stub
}

//create carrier robot and status
Tractor tractor;

/*
 * Wrapper method for the callBackStageOdm method
 */
void callBackStageOdm(const nav_msgs::Odometry msg){
	tractor.stageOdom_callback(msg);
}

void receiveTractorControl(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("tractor echoing tractor control: %s",msg->data.c_str());
	if (strcmp(msg->data.c_str(), "left") == 0) {
	    tractor.setLin(0);
	    tractor.setAng(0.2);
	} else if (strcmp(msg->data.c_str(), "right") == 0) {
	    tractor.setLin(0);
	    tractor.setAng(-0.2);
	} else if (strcmp(msg->data.c_str(), "up") == 0) {
	    tractor.setLin(1);
	    tractor.setAng(0);
	} else if (strcmp(msg->data.c_str(), "down") == 0) {
	    tractor.setLin(-1);
	    tractor.setAng(0);
	} else {
	    tractor.setLin(0);
	    tractor.setAng(0);
	}
}


int main(int argc, char **argv)
{
   // for (std::string line; std::getline(std::cin, line);) {
   //     ROS_WARN(line);
    //}

	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Tractor");
    
    // convert input parameters for Robot initialization from String to respective types
    //std::string xString = argv[1];
    //std::string yString = argv[2];
    //double xPos = atof(xString.c_str());
    //double yPos = atof(yString.c_str());
    //ROS_INFO("x start: %f", xPos);
    //ROS_INFO("y start: %f", yPos);
    
    //initialize the Carrier robot with the correct position, velocity and state parameters.
	tractor=Tractor(0,0,0,0,0);

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;


	ros::Rate loop_rate(10);
    tractor.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	//Broadcast the node's status information for other to subscribe to.
	//ros::Publisher pub=n.advertise<se306project::carrier_status>("status",1000);

	//subscribe to listen to messages coming from stage for odometry (this is base pose so it is
	//relative to the absolute frame of the farm.
	tractor.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000, callBackStageOdm);
	
	//subscribe to topic for manual control
	ros::Subscriber mysub_object = n.subscribe("/tractor",100,receiveTractorControl);

	//a count of how many messages we have sent
	//int count = 0;
	//carrier status message initialisation
	//se306project::carrier_status status_msg;

	//ROS loop
	while (ros::ok())
	{
		ros::spinOnce();
		//status_msg.my_counter=count;		//add counter to message
		//status_msg.status=carrierRobot.getStatus();		//add status to message
		//status_msg.pos_x=carrierRobot.getX(); //add x to message to broadcast
		//status_msg.pos_y=carrierRobot.getY();//add y to message to broadcast
		//status_msg.pos_theta=carrierRobot.getTheta(); //add angle to message to broadcast
		//status_msg.obstacle = obstacleStatus;
		//pub.publish(status_msg);	//publish message
		//carrierRobot.stateLogic();
		tractor.updateOdometry();
		loop_rate.sleep();
		//++count; // increase counter
	}
	return 0;
}

