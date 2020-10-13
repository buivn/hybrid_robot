#include <ros/ros.h>
#include <tf/transform_listener.h>
 
int main(int argc, char** argv){
  ros::init(argc, argv, "tf_processing");
 
  ros::NodeHandle node; 
  // ros::service::waitForService("spawn");
  // ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("spawn");
  // turtlesim::Spawn srv;
  // add_turtle.call(srv);
  // ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

  tf::TransformListener listener;
 
  ros::Rate rate(1.0);
  
  while (node.ok())
  {
    tf::StampedTransform transform;
    try
    {
       listener.lookupTransform("/base_link", "/target_pose_frame", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    // geometry_msgs::Pose target_pose1;
    geometry_msgs::Transform trans;
 
    // target_pose1.orientation.x = 0.5;
    // target_pose1.orientation.y = 0.5;
    // target_pose1.orientation.z = 0.5;
    // target_pose1.orientation.w = 0.5;
    // target_pose1.position.x = transform.getOrigin().x();
    // target_pose1.position.y = transform.getOrigin().y();
    // target_pose1.position.z = transform.getOrigin().z();
    tf::transformTFToMsg(transform, trans);

    
    std::cout << trans.translation.x << std::endl;
    std::cout << trans.translation.y << std::endl;
    std::cout << trans.translation.z << std::endl;
    std::cout << trans.rotation.x << std::endl;
    std::cout << trans.rotation.y << std::endl;
    std::cout << trans.rotation.z << std::endl;
    std::cout << trans.rotation.w << std::endl;
    // std::cout << transform.getBasis() << std::endl;
    // std::cout << transform.getBasis().z() << std::endl;
    // std::cout << transform.getBasis().w() << std::endl;
 

    rate.sleep();
  }
  return 0;
};