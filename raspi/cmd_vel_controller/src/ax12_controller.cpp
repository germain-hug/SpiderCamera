#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <stdio.h>
#include <termio.h>
#include <unistd.h>
//#include <ax12.h>
//#include <BioloidController.h>
//#include <dynamixel.h>

// Control table address
#define P_CW_ANGLE_L 6
#define P_CCW_ANGLE_L 8
#define P_GOAL_POSITION_L 30
#define P_MOVING_SPEED_L 32
#define P_PRESENT_POSITION_L 36
#define P_PRESENT_SPEED_L 38
#define P_MOVING 46

// Default setting
#define DEFAULT_BAUDNUM 1 // 1Mbps
#define DEFAULT_ID 1 //Motor ID



void callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{

       int GoalPos[2] = {0, 1023};
       //int GoalPos[2] = {0, 4095}; // for Ex series
       int index = 0;
       int Moving, PresentPos;
       int CommStatus;

	//BioloidController bioloid = BioloidController(1000000);
	// Set to Wheel Mode
 	//ax12SetRegister2(1,6,0);
	//delay(33);
	//ax12SetRegister2(1,8,0);
	//delay(33);

/*
  // Initialize AX-12W Connection
  if( dxl_initialize(DEFAULT_ID, DEFAULT_BAUDNUM) == 0 ){
    printf( "Failed to open USB2Dynamixel!\n" );
    printf( "Press Enter key to terminate...\n" );
    getchar();
    return 0;
  } else {
    printf( "Succeed to open USB2Dynamixel!\n" );
  }


  ros::init(argc, argv, "ax12_controller");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("topic_name", 1000, callback);
  ros::spin();*/
  return 0;
}

