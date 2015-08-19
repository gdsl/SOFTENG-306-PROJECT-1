#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include "Person.h"
#include "AlphaPerson.h"
#include "se306project/robot_status.h"
#include "se306project/human_status.h"
#include <math.h>
 
AlphaPerson::AlphaPerson() : Person() {

}

AlphaPerson::AlphaPerson(double x, double y): Person(x,y) {
}

AlphaPerson::~AlphaPerson() {

}

AlphaPerson alphaPerson(-30.00,21.15);
bool foundTree = false;
bool isSearch = false;
double yDistance = 0;
// Default human behaviour = walking
std::string status="Walking";

// Keeps track of current position that human is facing
double radians;
double angle;

void stage_positionCallback(nav_msgs::Odometry msg) {
    alphaPerson.stageOdom_callback(msg);
}

void stage_laserCallback(sensor_msgs::LaserScan msg) {
    alphaPerson.stageLaser_callback(msg);

    if (isSearch) {
        int l=msg.intensities.size();
        double minDist = 1000;
        std::vector<double> v;
        std::map<double,double> Xmap;
        std::map<double,double> Ymap;
        for (int i = 0; i<l; i++) {
            if (msg.intensities[i] == 1) { 
                double toDegree =  alphaPerson.getTheta()*180/M_PI;
                double absAngle = i + toDegree - 90;
                double obsX = alphaPerson.getX() + msg.ranges[i]*cos(absAngle*M_PI/180);
                double obsY = alphaPerson.getY() + msg.ranges[i]*sin(absAngle*M_PI/180);

                if (msg.ranges[i] > 1.5 && -105 < absAngle && absAngle < -90) {
                    foundTree = true;
                    if (msg.ranges[i] < minDist) minDist = msg.ranges[i];
                    v.push_back(msg.ranges[i]);
                    Xmap[msg.ranges[i]] = obsX;
                    Ymap[msg.ranges[i]] = obsY;
                //ROS_INFO("ALPHA PERSON OBS dist:%f angle:%d abs:%f obsX:%f obsY:%f robotAngle:%f", msg.ranges[i], i,absAngle, obsX,obsY,toDegree);
                    
                }
            }
        }

        if (foundTree) {
            int count = 0;
            double sumX = 0;
            double sumY = 0;        
            
            for (std::vector<double>::iterator it = v.begin(); it != v.end(); ++it) {
                if (minDist <= *it && *it <= minDist+0.2) {
                    count++;
                    sumX = sumX + Xmap[*it];
                    sumY = sumY + Ymap[*it];
                }
            }

            double avgX = 0;
            double avgY = 0;

            if (count > 0) {
                avgX = sumX / count;
                avgY = (sumY /count) - 0.25;
                yDistance = avgY - alphaPerson.getY() + 0.75;
            }
            ROS_INFO("ALPHA PERSON x:%f y:%f",avgX,avgY);
        }
    }
}

int main(int argc, char **argv) 
{
    
    
    //initialise ros    
    ros::init(argc,argv,"AlphaPerson");

    //create ros handler for this node
    ros::NodeHandle n;
    
    alphaPerson.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	ros::Publisher pub = n.advertise<se306project::human_status>("status",100);

    alphaPerson.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000,stage_positionCallback);
    alphaPerson.baseScan_Sub = n.subscribe<sensor_msgs::LaserScan>("base_scan", 1000,stage_laserCallback);
    ros::Rate loop_rate(10); 
    int state = 0;
	int count = 0;
	se306project::human_status status_msg;
    int none = -1;
    int trimming_tree = 0;
    int moving_to_search_spot = 1;
    int searching = 2;
    int go_to_next_tree = 3;
    while (ros::ok())
    { 
        alphaPerson.move();
        
        if (alphaPerson.getMovementQueueSize() == 0) {
            if (state == trimming_tree) {
                alphaPerson.faceSouth(1);
                state = moving_to_search_spot;

            } else if (state == moving_to_search_spot) {
                alphaPerson.faceEast(1);
                alphaPerson.addMovement("forward_x",0.75,1);
                alphaPerson.faceSouth(1);
                state = searching;

            } else if (state == searching) {
                isSearch = true;
                if (foundTree) state = go_to_next_tree;

            } else if (state == go_to_next_tree) {
                isSearch = false;
                foundTree = false;
                alphaPerson.addMovement("forward_y",yDistance,1); //move down
                alphaPerson.faceWest(1);
                alphaPerson.addMovement("forward_x",-0.75,1);
                alphaPerson.faceSouth(1);
                
                state = trimming_tree;
            }
        }

     

    /*
     if (alphaPerson.getMovementQueueSize() == 0 && state == 0) {

            alphaPerson.faceEast(1);
            alphaPerson.addMovement("forward_x", 35, 1);
            alphaPerson.faceSouth(1);
            alphaPerson.faceEast(1);
            alphaPerson.addMovement("forward_x", 35, 1);
            alphaPerson.faceSouth(1);
            state = 1;
            
	} else if (alphaPerson.getMovementQueueSize() == 0 && state == 1) {
            
            alphaPerson.faceWest(1);
            alphaPerson.addMovement("forward_x", -35, 1);
            alphaPerson.faceSouth(1);
            alphaPerson.faceWest(1);
            alphaPerson.addMovement("forward_x", -35, 1);
            alphaPerson.faceSouth(1);
           state = 0;
	} */
    
	//assign to status message
	status_msg.my_counter = count++;//add counter to message to broadcast
	status_msg.status=status;//add status to message to broadcast
	status_msg.pos_x=alphaPerson.getX(); //add x to message to broadcast
	status_msg.pos_y=alphaPerson.getY();//add y to message to broadcast
	status_msg.pos_theta=alphaPerson.getTheta(); //add angle to message to broadcast
	pub.publish(status_msg);//publish the message for other node
        
        ros::spinOnce();
        loop_rate.sleep();

	// ******** MOVE THIS FUNCTION TO ENTITY - REFACTOR **************
	// Logic to determine current status of Human - Walking/Idle/Turning
	// Convert radians to degrees
	radians = alphaPerson.getTheta();
	angle = roundf(radians * 57.2957795 * 100) / 100;
	// Check if human is moving (and therefore 'walking')
	if (alphaPerson.getLin() > 0.01) {
		status = "Walking";
	}
	// Check if human is facing North/East/South/West AND not moving (and therefore 'idle')
	else if ((angle == -360) || (angle == -270) || (angle == -180) || (angle == -90) || (angle == 0) || (angle == 90) || (angle == 180) || (angle == 270) || (angle == 360) && (alphaPerson.getLin() == 0)) {
		status = "Idle";
	}
	else {
		status = "Turning";
	}
    }
    return 0;
}
