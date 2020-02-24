#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <sstream>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <stdio.h>      /* printf, fgets */
#include <stdlib.h>     /* atoi */
#include "climb3/DynamixelCommand.h"
#include <string>
#include <Eigen/Dense>
#include <map>
using namespace std;

serial::Serial serialWheel1;
serial::Serial serialWheel2;
serial::Serial serialUpDown1;
serial::Serial serialUpDown2;
string  wheel1_port="/dev/ttyUSB2";
string  wheel2_port="/dev/ttyUSB4";
string  updown1_port="/dev/ttyUSB1";
string  updown2_port="/dev/ttyUSB3";

// Init variables

char key(' ');
double speed_robot=20;

int32_t servo1_val1=0;  // joint 6
int32_t servo1_val2=0;  // joint 5
int32_t servo1_val3=276000; // joint 4
int32_t servo1_val4=-135000;  // joint 3
int32_t servo1_val5=0;      // joint 2
int32_t servo1_val6=85000;  // joint 1

int32_t targetJointPos1=0;
int32_t targetJointPos2=0;
int32_t targetJointPos3=0;
int32_t targetJointPos4=0;
int32_t targetJointPos5=0;
int32_t targetJointPos6=0;

bool change = true;

// For non-blocking keyboard inputs - get a char from keyboard
int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);
  // Get the current character
  ch = getchar();
  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

// send Data to a motor
void sendData(int serialport, string A,double x){
    string Data= string(A+to_string(x)+'?');
    int n = Data.length(); 
    char data[n + 1]; 
    strcpy(data, Data.c_str()); 
    if(serialport==1) serialWheel1.write(data);
    else if(serialport==2) serialWheel2.write(data);
    else if(serialport==3) serialUpDown1.write(data);
    else if(serialport==4) serialUpDown2.write(data);
}

void move_lui()
{
  sendData(1,"L",speed_robot);
  sendData(1,"R",speed_robot);
  sendData(2,"L",speed_robot);
  sendData(2,"R",speed_robot);
}
void move_toi()
{
  sendData(1,"L",-speed_robot);
  sendData(1,"R",-speed_robot);
  sendData(2,"L",-speed_robot);
  sendData(2,"R",-speed_robot);
}
void move_phai()
{
  sendData(1,"L",-speed_robot);
  sendData(1,"R",speed_robot);
  sendData(2,"L",-speed_robot);
  sendData(2,"R",speed_robot);
}
void move_trai()
{
  sendData(1,"L",speed_robot);
  sendData(1,"R",-speed_robot);
  sendData(2,"L",speed_robot);
  sendData(2,"R",-speed_robot);
}
void stop()
{
  sendData(1,"L",0);
  sendData(1,"R",0);
  sendData(2,"L",0);
  sendData(2,"R",0);
}



int setupserial()
{
    try
    {
        serialUpDown1.setPort(updown1_port);
        serialUpDown1.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serialUpDown1.setTimeout(to);
        serialUpDown1.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port updown1 ");
        return -1;
    }

    if(serialUpDown1.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    try
    {
        serialUpDown2.setPort(updown2_port);
        serialUpDown2.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serialUpDown2.setTimeout(to);
        serialUpDown2.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port updown2 ");
        return -1;
    }

    if(serialUpDown2.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }
    try
    {
        serialWheel1.setPort(wheel1_port);
        serialWheel1.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serialWheel1.setTimeout(to);
        serialWheel1.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port wheel1_port");
        return -1;
    }

    if(serialWheel1.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    try
    {
        serialWheel2.setPort(wheel2_port);
        serialWheel2.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serialWheel2.setTimeout(to);
        serialWheel2.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port wheel2_port");
        return -1;
    }

    if(serialWheel2.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }
}
void jointState_wormingCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  // Eigen::VectorXd jointstate;
  std::cout << "receiving the joint position - outside the loop" << std::endl;
  for (int i=0; i<6;i++)
  {
    if (i==0) targetJointPos1 = int32_t(msg->data[i]*500000/3.141592653);
    if (i==1) targetJointPos2 = int32_t(msg->data[i]*500000/3.141592653);
    if (i==2) targetJointPos3 = int32_t(msg->data[i]*500000/3.141592653);
    if (i==3) targetJointPos4 = int32_t(msg->data[i]*500000/3.141592653);
    if (i==4) targetJointPos5 = int32_t(msg->data[i]*500000/3.141592653);
    if (i==5) targetJointPos6 = int32_t(msg->data[i]*500000/3.141592653);
    // targetJointPos3 = 5000;
    // targetJointPos4 = -5000;
    std::cout << "receiving the joint position" << std::endl;
  }
  // targetJointPos2 = 0;
  // targetJointPos3 = 0;
  // targetJointPos5 = 0;
  // targetJointPos6 = 0;
  // targetJointPos1 = 0;
  change = true;
}


int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "Robot_control");
  ros::NodeHandle nh;
  
  // Init cmd_vel publisher
  // ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  // geometry_msgs::Twist twist;
  ros::ServiceClient client = nh.serviceClient<climb3::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
  climb3::DynamixelCommand srv;
  setupserial();
  ros::Rate loop_rate(25);
  // int mode=0;
  
  ros::Subscriber tra_sub = nh.subscribe("/trajectory_processing/jointState_worming", 5, jointState_wormingCallback);
  bool worming_mode = true;
  bool mobile_mode = false;
  bool switching_controller = true;
  bool area_check = false, plane_check = false, height = false;

  while(ros::ok())
  {

    // the switching controller
    // if (switching_controller)
    // {
    //   // asking service to process the point cloud to return area_check, plane_check, height_check
    //   bool switching = false;
    //   bool keep_working = false;
    //   keep_working = area_check*plane_check;
    //   if (keep_working)
    //   {
    //     switching = height;
    //     if (switching)
    //       mobile_mode = true;
    //     else
    //       worming_mode = true;
    //   }
    // }

    if (mobile_mode)
    {
      std::cout << "The mobile mode is turned on " << std::endl;
      switching_controller = true;
      mobile_mode = false;
      // asking a service to take a point cloud data
      // taking a picture - asking a service

    }
    if (worming_mode)
    {
      // asking a service to have a pointcloud  --------------------------------------------
      // asking another service to process a point cloud to have Pose ---------------------------------
      bool convPose_state = true, move_1leg = false, move_2leg = false;
      Eigen::MatrixXd trajectory1(6,6);
      trajectory1(0,0) = -1617;
      trajectory1(0,1) = -25;
      trajectory1(0,2) = 2948;
      trajectory1(0,3) = -2207;
      trajectory1(0,4) = 49;
      trajectory1(0,5) = -1577;

      trajectory1(1,0) = -15011;
      trajectory1(1,1) = -236;
      trajectory1(1,2) = 27365;
      trajectory1(1,3) = -20488;
      trajectory1(1,4) = 456;
      trajectory1(1,5) = -14638;

      trajectory1(2,0) = -41203;
      trajectory1(2,1) = -648;
      trajectory1(2,2) = 75114;
      trajectory1(2,3) = -56237;
      trajectory1(2,4) = 1252;
      trajectory1(2,5) = -40179;
      
      trajectory1(3,0) = -67393;
      trajectory1(3,1) = -1060;
      trajectory1(3,2) = 122857;
      trajectory1(3,3) = -91982;
      trajectory1(3,4) = 2049;
      trajectory1(3,5) = -65718;

      trajectory1(4,0) = -93454;
      trajectory1(4,1) = -1471;
      trajectory1(4,2) = 170366;
      trajectory1(4,3) = -127553;
      trajectory1(4,4) = 2841;
      trajectory1(4,5) = -91131;

      trajectory1(5,0) = -104507;
      trajectory1(5,1) = -1645;
      trajectory1(5,2) = 190515;
      trajectory1(5,3) = -142639;
      trajectory1(5,4) = 3177;
      trajectory1(5,5) = -101909;

      int32_t convPos_joint1=0, convPos_joint3=-30000, convPos_joint4=60000;

      
      // move down permanent magnet of the second leg ---------------------------------------
      // move up permanent magnet of the first leg -------------------------------------------

      // moving to the convenient Pose
      if (convPose_state)
      {
        srv.request.command="";
        srv.request.addr_name="Goal_Position";
        // moving the joint 1 the base joint - first
        srv.request.id=6;
        srv.request.value = convPos_joint1;
        client.call(srv);
        //moving the joint 3 - the second
        srv.request.id=4;
        srv.request.value = convPos_joint3;
        client.call(srv);
        //moving the joint 4 - the last
        srv.request.id=3;
        srv.request.value = convPos_joint4;
        client.call(srv);
        // move_1leg = true;
        convPose_state = false;
      }    

      if (move_1leg)
      {
        for (int k = 0; k<6; k++)
          for (int j = 0; j < 6; j++)
          {
            srv.request.command="";
            srv.request.id=j+1;
            srv.request.addr_name="Goal_Position";

            if(j==0) srv.request.value = trajectory1(k,5);  // joint 6   - counter clockwise - positive
            if(j==1) srv.request.value = trajectory1(k,4);  // joint 5  - counter clockwise - positive
            if(j==2) srv.request.value = -trajectory1(k,3); // joint 4   - clockwise - positive
            if(j==3) srv.request.value = trajectory1(k,2);  // joint 3   - counter clockwise - positive
            if(j==4) srv.request.value = trajectory1(k,1);  // joint 2    - counter clockwise - positive
            if(j==5) srv.request.value = -trajectory1(k,0); // joint 1    - clockwise - positive
            client.call(srv);
          }
        move_2leg = true;
        move_1leg = false;       
      }
      
      // move down the permanent magnet of the first leg ---------------------------------------------------
      // move up the permanent magnet of the second leg ----------------------------------------------------

      // moving to the initial Pose
      if (move_2leg)
      {
        srv.request.command="";
        srv.request.addr_name="Goal_Position";
        // moving the joint 1 the base joint - first
        srv.request.id=6;
        srv.request.value = servo1_val6;
        client.call(srv);
        //moving the joint 3 - the second
        srv.request.id=4;
        srv.request.value = servo1_val4;
        client.call(srv);
        //moving the joint 4 - the last
        srv.request.id=3;
        srv.request.value = servo1_val3;
        client.call(srv);
        move_2leg = false;
      }
      switching_controller = true;
      worming_mode = false;

      // move down the permanent magnet of the second leg ---------------------------------------------------
    }

    // if (change)
    // {
    //   // std::cout << "Is there any time the algorithm go inside here??"  << std::endl;
    //   for (int j = 0; j < 6; j++)
    //   {
    //     srv.request.command="";
    //     srv.request.id=j+1;
    //     srv.request.addr_name="Goal_Position";

    //     if(j==0) srv.request.value = targetJointPos6;  // joint 6   - counter clockwise - positive
    //     if(j==1) srv.request.value = targetJointPos5;  // joint 5  - counter clockwise - positive
    //     if(j==2) srv.request.value = -targetJointPos4; // joint 4   - clockwise - positive
    //     if(j==3) srv.request.value = targetJointPos3;  // joint 3   - counter clockwise - positive
    //     if(j==4) srv.request.value = targetJointPos2;  // joint 2    - counter clockwise - positive
    //     if(j==5) srv.request.value = -targetJointPos1; // joint 1    - clockwise - positive
        
    //     // if(j==5) srv.request.value = 85000;   // joint 1    - clockwise - positive
    //     // if(j==4) srv.request.value = 0;       // joint 2    - counter clockwise - positive
    //     // if(j==3) srv.request.value = -135000;  // joint 3   - counter clockwise - positive
    //     // if(j==2) srv.request.value = 276000;  // joint 4    - clockwise - positive
    //     client.call(srv);
    //   }
    //   change = false;
    // }


    
    ros::spinOnce();
    loop_rate.sleep();

  }
  // ros::spinOnce();

  return 0;
}
