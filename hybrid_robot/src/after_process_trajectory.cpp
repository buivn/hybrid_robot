#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


#include <stdio.h>

#include <sstream>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>
#include <stdio.h>      /* printf, fgets */
#include <stdlib.h>     /* atoi */
#include <string>
#include <Eigen/Dense>
#include <map>
using namespace std;

// Init variables
bool change = false;

void jointState_wormingCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  // Eigen::VectorXd jointstate;

  cout << "receiving the data  ------------" << endl;
  for (int i=0; i<6;i++)
  {
    if (i==0) cout << int32_t(msg->data[i]*500000/3.141592653) << endl;
    if (i==1) cout << int32_t(msg->data[i]*500000/3.141592653) << endl;
    if (i==2) cout << int32_t(msg->data[i]*500000/3.141592653) << endl;
    if (i==3) cout << int32_t(msg->data[i]*500000/3.141592653) << endl;
    if (i==4) cout << int32_t(msg->data[i]*500000/3.141592653) << endl;
    if (i==5) cout << int32_t(msg->data[i]*500000/3.141592653) << endl;
  }
  change = true;
}


int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "Robot_control");
  ros::NodeHandle nh;
  ros::Rate loop_rate(100);
  //ros::ServiceClient client = nh.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("dynamixel_workbench_msgs/DynamixelCommand");
  // Init cmd_vel publisher
  
  ros::Subscriber tra_sub = nh.subscribe("/trajectory_processing/jointState_worming", 5, jointState_wormingCallback);
  // ros::Publisher jp_pub = node_handle.advertise<std_msgs::Float64MultiArray>("trajectorySet", 1);

  while(ros::ok())
  {

    if (change)
    {
      change = false;
      cout << "receiving the data in the MAIN function" << endl;
    }

    // Publish it and resolve any remaining callbacks

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
