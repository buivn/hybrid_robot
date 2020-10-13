#include <ros/ros.h>
#include <stdio.h>

#include <tf/transform_listener.h>
#include <Eigen/Dense>


int main(int argc, char** argv)
{ 
  ros::init(argc, argv, "my_tf_listener");
  ros::NodeHandle node("~");
  //ros::NodeHandle node;

  tf::TransformListener listener;
  ros::Rate rate(1);
  float r_x = 0.0f;
  float r_y = 0.0f;
  float r_z = 0.0f;
  float r_w = 0.0f;

  float t_x = 0.0f;
  float t_y = 0.0f;
  float t_z = 0.0f;
  int count = 0;

  bool cali;
  int caliStep;
  node.param("calibration", cali, true);
  node.param("calibrationStep", caliStep, 1);
  // std::cout << "The value of calibration " << cali << std::endl;

  //while (node.ok())
  while (ros::ok())
  {
      // hold the transformation parameters between two coordinate frames 
      //tf::StampedTransform result_cali; 
      tf::StampedTransform transformPara;
      try
      { 
        if (cali)
        {
          if (caliStep == 1)
          {
            // listener.waitForTransform("inter_robotBase_frame", "endEffectorI5_frame", ros::Time(0), ros::Duration(3.0));
            listener.waitForTransform("base_link", "camera_frame1", ros::Time(0), ros::Duration(4.0));
            // For calibration - calculate the transformation between the two frames
            listener.lookupTransform("camera_frame1", "base_link", ros::Time(0), transformPara);
            // listener.lookupTransform("base_link", "camera_frame1", ros::Time(0), transformPara);

            // get the value
            tf::Quaternion rotation = transformPara.getRotation();
            tf::Vector3 tfVect = transformPara.getOrigin();

            std::cout << "Calibration at Step 1 - Inverse Quaternion from Auto I5 Robot" << std::endl;
            ROS_INFO("Origin & Quat: %f %f %f %f %f %f %f", tfVect.x(), tfVect.y(), tfVect.z(), rotation.x(), rotation.y(), rotation.z(), rotation.w());
 
            
            tf::Matrix3x3 mat(rotation);
            tfScalar y, p, r;
            mat.getEulerYPR(y, p, r);

            ROS_INFO("Three Angles: %f %f %f ", r, p, y);


            // btQuaternion::btMatrix3x3 tfMatrix; // = transformPara.getBasis();

            // geometry_msgs::Pose robot_pose;
            Eigen::Matrix4f hete_matrix;
            
            // for translation
            hete_matrix(0,3) = tfVect.x();
            hete_matrix(1,3) = tfVect.y();
            hete_matrix(2,3) = tfVect.z();
            hete_matrix(3,3) = 1;
            // the last row
            hete_matrix(3,0) = 0;
            hete_matrix(3,1) = 0;
            hete_matrix(3,2) = 0;
            
            // for rotation part
            hete_matrix(0,0) = 1 - 2*pow(rotation.y(),2) - 2*pow(rotation.z(),2);
            hete_matrix(1,0) = 2*(rotation.x()*rotation.y() + rotation.z()*rotation.w());
            hete_matrix(2,0) = 2*(rotation.x()*rotation.z() - rotation.y()*rotation.w());

            hete_matrix(0,1) = 2*(rotation.x()*rotation.y() - rotation.z()*rotation.w());
            hete_matrix(1,1) = 1 - 2*pow(rotation.x(),2) - 2*pow(rotation.z(),2);
            hete_matrix(2,1) = 2*(rotation.x()*rotation.w() + rotation.y()*rotation.z());

            hete_matrix(0,2) = 2*(rotation.x()*rotation.z() + rotation.y()*rotation.w());
            hete_matrix(1,2) = 2*(rotation.y()*rotation.z() - rotation.x()*rotation.w());
            hete_matrix(2,2) = 1 - 2*pow(rotation.x(),2) - 2*pow(rotation.y(),2);



            ROS_INFO("Matrix: %f %f %f %f ", hete_matrix(0,0), hete_matrix(0,1), hete_matrix(0,2), hete_matrix(0,3));
            ROS_INFO("Matrix: %f %f %f %f ", hete_matrix(1,0), hete_matrix(1,1), hete_matrix(1,2), hete_matrix(1,3));
            ROS_INFO("Matrix: %f %f %f %f ", hete_matrix(2,0), hete_matrix(2,1), hete_matrix(2,2), hete_matrix(2,3));
            ROS_INFO("Matrix: %f %f %f %f ", hete_matrix(3,0), hete_matrix(3,1), hete_matrix(3,2), hete_matrix(3,3));
            
          



            // ROS_INFO("Matrix: %f %f %f ", tfMatrix.xz(), tfMatrix.yz(), tfMatrix.zz());
          }

          if (caliStep == 2)
          {
            count++;
            // wait until the two frames of camera and robot base are available
            listener.waitForTransform("camera_depth_optical_frame", "depend_robotBase_frame", ros::Time(0), ros::Duration(3.0));
            // For calibration - calculate the transformation between the two frames
            listener.lookupTransform("camera_depth_optical_frame", "depend_robotBase_frame", ros::Time(0), transformPara);

            // get the angle/quaternion
            tf::Quaternion rotation = transformPara.getRotation();
            // get the distance between the two frames
            tf::Vector3 tfVect = transformPara.getOrigin();

            r_x += rotation.x();
            r_y += rotation.y();
            r_z += rotation.z();
            r_w += rotation.w();
            t_x += tfVect.x();
            t_y += tfVect.y();
            t_z += tfVect.z();

            ROS_INFO("Gathering data %d", count);
            if (count == 100)
            {
                std::cout << "Calibration at Step 2 - Quaternion of robotBase frame comapring to camera_frame" << std::endl;
                //ROS_INFO("Quat: %f %f %f %f", );
                ROS_INFO("Origin & Quat: %f %f %f %f %f %f %f", t_x/float(count), t_y/float(count), t_z/float(count), r_x/float(count), r_y/float(count), r_z/float(count), r_w/float(count));
                count = 0;
                t_x = 0.0;
                t_y = 0.0;
                t_z = 0.0;
                r_x = 0.0;
                r_y = 0.0;
                r_z = 0.0;
                r_w = 0.0;
            }
          }
        }
        else
        {
          // For xtion_frame.txt & displaying axes in OpenRave
          // wait for two frames: estimate_robot_base_frame and grasp_aubo_frame available
          listener.waitForTransform("fix_robotBase_frame", "grasp_aubo_frame", ros::Time(0), ros::Duration(3.0));        

          // transform the coordiantes from grasp_aubo_frame to estimate_robot_base_frame
          // For display grasp in OpenRave
          listener.lookupTransform("fix_robotBase_frame", "grasp_aubo_frame", ros::Time(0), transformPara);  

          // get the value
          tf::Quaternion rotation = transformPara.getRotation();
          tf::Vector3 tfVect = transformPara.getOrigin();

          ROS_INFO("Origin & Quat: %f %f %f %f %f %f %f", tfVect.x(), tfVect.y(), tfVect.z(), rotation.x(), rotation.y(), rotation.z(), rotation.w());
        }
      }
      catch (tf::TransformException &ex) 
      {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }
      
      rate.sleep();
  }
  return 0;
}