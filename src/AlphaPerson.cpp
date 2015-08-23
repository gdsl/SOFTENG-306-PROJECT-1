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
    this->tickCount = 0;
}

AlphaPerson::~AlphaPerson() {

}

AlphaPerson::State AlphaPerson::getState() {
    return this->state;
}

void AlphaPerson::setState(State s) {
    this->state = s;
}

AlphaPerson alphaPerson(-30.00,21.15);
bool foundTree = false;
bool isSearch = false;
bool lookAtBottom = true;
double yDistance = 0;
double xDistance = 0;
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
                
                bool whereToLook;
                if (lookAtBottom) {
                    whereToLook = -105 < absAngle && absAngle < -90;
                } else {
                    whereToLook = 90 < absAngle && absAngle < 105;
                }

                if (msg.ranges[i] > 1.5 && whereToLook) {
                    foundTree = true;
                    if (msg.ranges[i] < minDist) minDist = msg.ranges[i];
                    v.push_back(msg.ranges[i]);
                    Xmap[msg.ranges[i]] = obsX;
                    Ymap[msg.ranges[i]] = obsY;
                ROS_INFO("ALPHA PERSON OBS dist:%f angle:%d abs:%f obsX:%f obsY:%f robotAngle:%f", msg.ranges[i], i,absAngle, obsX,obsY,toDegree);
                    
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
                avgY = (sumY /count) ;
                if (lookAtBottom) {
                    avgY -= 0.25;
                } else {
                    avgY += 0.25;
                }
                xDistance = avgX - alphaPerson.getX();
                yDistance = avgY - alphaPerson.getY() + 0.75;
            }            
            ROS_INFO("ALPHA PERSON x:%f y:%f xDist:%f yDist:%f",avgX,avgY,xDistance,yDistance);
        }
    }
}

void AlphaPerson::stateLogic() {
    if (alphaPerson.getState() == TRIMMING ) {
        alphaPerson.setStatus("Trimming");
        if (alphaPerson.getMovementQueueSize() == 0) { 
            alphaPerson.faceEast(1);
            alphaPerson.addMovement("forward_x",0.75,1);
            if (lookAtBottom) {
               alphaPerson.faceSouth(1);
            } else {
                alphaPerson.faceNorth(1);
            }
            alphaPerson.setState(MOVING_TO_SEARCH_SPOT);
        }
    } else if (alphaPerson.getState() == MOVING_TO_SEARCH_SPOT ) {
        alphaPerson.setStatus("Moving to search spot");
        if (alphaPerson.getMovementQueueSize() == 0) {
            alphaPerson.setState(SEARCHING);            
        }
    } else if (alphaPerson.getState() == SEARCHING) {
        alphaPerson.setStatus("Searching for next tree");
        isSearch = true;
        if (alphaPerson.getMovementQueueSize() == 0) {
            if (foundTree) {
                alphaPerson.setState(GO_TO_NEXT_TREE);
                isSearch = false;
                foundTree = false;
                if (yDistance >= 0) {
                    alphaPerson.faceNorth(1);
                } else {
                    alphaPerson.faceSouth(1);
                }
                alphaPerson.addMovement("forward_y",yDistance,1); //move down or up
                alphaPerson.faceWest(1);
                alphaPerson.addMovement("forward_x",xDistance,1);
                alphaPerson.faceSouth(1);
            } else {
                
                if (this->tickCount > 20)  {
                   this->tickCount = 0;
                    lookAtBottom = !lookAtBottom;
                    if (lookAtBottom) {
                        alphaPerson.faceSouth(1);
                    } else {
                        alphaPerson.faceNorth(1);
                    }
                }
                this->tickCount++;
            }

        }

    } else if (alphaPerson.getState() == GO_TO_NEXT_TREE) {
        alphaPerson.setStatus("Moving to next tree");
        if (alphaPerson.getMovementQueueSize() == 0) {
            alphaPerson.setState(TRIMMING);            
        }
    }
}



int main(int argc, char **argv) 
{
    
    
    //initialise ros    
    ros::init(argc,argv,"AlphaPerson");

    // convert input parameters for person initialization from String to respective types
    std::string xString = argv[1];
    std::string yString = argv[2];
    double xPos = atof(xString.c_str());
    double yPos = atof(yString.c_str());
    
    alphaPerson = AlphaPerson(xPos,yPos);
    
    alphaPerson.setState(AlphaPerson::MOVING_TO_SEARCH_SPOT);
    alphaPerson.setStatus("Initial");
    //create ros handler for this node
    ros::NodeHandle n;
    
    alphaPerson.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	ros::Publisher pub = n.advertise<se306project::human_status>("status",1000);

    alphaPerson.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000,stage_positionCallback);
    alphaPerson.baseScan_Sub = n.subscribe<sensor_msgs::LaserScan>("base_scan", 1000,stage_laserCallback);
    ros::Rate loop_rate(10); 
    int state = 2;
	int count = 0;
	se306project::human_status status_msg;
    int none = -1;
    int trimming_tree = 0;
    int moving_to_search_spot = 1;
    int searching = 2;
    int go_to_next_tree = 3;
    int tickCount = 0;
   
    while (ros::ok())
    { 
         alphaPerson.move();
         alphaPerson.stateLogic();
       
        /*
        if (alphaPerson.getMovementQueueSize() == 0) {
            if (state == trimming_tree) {
                alphaPerson.faceSouth(1);
                state = moving_to_search_spot;

            } else if (state == moving_to_search_spot) {
                alphaPerson.faceEast(1);
                alphaPerson.addMovement("forward_x",0.75,1);
                if (lookAtBottom) {
                    alphaPerson.faceSouth(1);
                } else {
                    alphaPerson.faceNorth(1);
                }
                state = searching;

            } else if (state == searching) {
                isSearch = true;
                if (foundTree) {
                    state = go_to_next_tree;
                } else {
                    if (tickCount > 10)  {
                        tickCount = 0;
                        lookAtBottom = !lookAtBottom;
                        if (lookAtBottom) {
                            alphaPerson.faceSouth(1);
                        } else {
                            alphaPerson.faceNorth(1);
                        }
                    }
                    tickCount++;
                }

            } else if (state == go_to_next_tree) {
                isSearch = false;
                foundTree = false;
                alphaPerson.addMovement("forward_y",yDistance,1); //move down or up
                alphaPerson.faceWest(1);
                alphaPerson.addMovement("forward_x",xDistance,1);
                alphaPerson.faceSouth(1);
                
                state = trimming_tree;
            }
        }*/
    
	    //assign to status message
	    status_msg.my_counter = count++;//add counter to message to broadcast
	    status_msg.status=alphaPerson.getStatus();//add status to message to broadcast
	    status_msg.pos_x=alphaPerson.getX(); //add x to message to broadcast
	    status_msg.pos_y=alphaPerson.getY();//add y to message to broadcast
	    status_msg.pos_theta=alphaPerson.getTheta(); //add angle to message to broadcast
	    pub.publish(status_msg);//publish the message for other node
        
        ros::spinOnce();
        loop_rate.sleep();
        alphaPerson.determineStatus();
    }
    return 0;
}
