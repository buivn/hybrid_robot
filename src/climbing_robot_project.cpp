//#include <gpg/cloud_camera.h>
#include <iostream>
#include <stdio.h>
#include "ros/ros.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include "sensor_msgs/PointCloud2.h"
#include "targetPlane/target_plane.h"
#include <math.h>
#include <string>


int main (int argc, char** argv)
{
    ros::init(argc, argv, "targetPlane");
    ros::NodeHandle nh;
    // declare a targetPlane class
    target_plane::targetPlane tarPlane;
    // ros::Subscriber sub = nh.subscribe("cloud_in", 1, &target_plane::targetPlane::pointcloud_Callback, &tarPlane);

    // pcl::PointCloud<pcl::PointXYZRGB> cloud, cloud2; //(new pcl::PointCloud<pcl::PointXYZ>);
    // std::string filename, filenamejpg, filenamepcd, root, fileAfterProcess; 
    // filename= "4objectnew1";
    
    // root = "/home/buivn/bui_IROS2020/src/pclFilCen/";
    // filenamepcd = root +filename+".pcd";
    ros::spin();

    return 0;

    



    // cloud = pcd_read(filenamepcd);
    // // show_pcd(cloud.makeShared(), 0, "Original point cloud data");

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudp1 (new pcl::PointCloud<pcl::PointXYZRGB>), cloudp4 (new pcl::PointCloud<pcl::PointXYZRGB>),
    //           cloudp2 (new pcl::PointCloud<pcl::PointXYZRGB>), cloudp3 (new pcl::PointCloud<pcl::PointXYZRGB>); 
    // cloudp1 = passthrough_filter(cloud.makeShared());
    // // show_pcd(cloudp1, 0, "After passThrough filtering");

    // // show_pcd only work with pointer->use cloud.makeShared() to return a pointer
    // // downsampling the data - cloudp1 is a pointer
    // cloudp1 = downsampling(cloudp1);
    // // show_pcd(cloudp1, 0, "After downsampling");
    // // fileAfterProcess = "afterprocess";
    // // pcd_write(filename, cloudp1);

    // // cloudp2 is a pointer -  planar filter
    // cloudp2 = get_planes(cloudp1);

    // show_pcd(cloudp2, 0, "After planar filtering");


    // // pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    // pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
    // cloud_normals = estimate_normal(cloudp2);
    // show_normals(cloudp2, cloud_normals);
    // Eigen::Vector3f ave = average_normals(cloud_normals);
    // std::cout << "the average normals: " << ave << std::endl;
    // Eigen::Vector4f centroid;
    // // calculate the centroid of the plane
    // pcl::compute3DCentroid(*cloudp2, centroid);
    // std::cout << "the centroid of the plane: " << centroid << std::endl;
    // show_aver_normals(cloudp2, ave, centroid);
    
    // float area;
    // area = calculateAreaPolygon(*cloudp2);
    // std::cout << "The area of the point cloud: " << area << std::endl;
    

    // std::vector<pcl::PointIndices> cluster_indices;
    // cluster_indices = extract_object(cloudp2);

    
    return (0);
}
