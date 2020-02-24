#ifndef TARGET_PLANE_H_
#define TARGET_PLANE_H_

#include <iostream>
#include <stdio.h>
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/don.h>

#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <Eigen/Dense>
#include "sensor_msgs/PointCloud2.h"
#include <math.h>
#include <string>
#include "ros/ros.h"


namespace target_plane
{
    class targetPlane
    {
        public:     // public function
            targetPlane();
            // ~targetPlane();
            void pointcloud_Callback(const sensor_msgs::PointCloud2& msg);
            void calculateAreaPolygon(const pcl::PointCloud<pcl::PointXYZRGB> &polygon);
            void readFromFile(const std::string& filename);
            void passthrough_filter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudp);
            void downsampling(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudp);
            void get_planes(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudp);
            void show_pcd(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudp, int clus_order, std::string& titlename);
            void show_centroid(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudp, Eigen::MatrixXf& centroid3D);
            void show_normals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudp, const pcl::PointCloud<pcl::PointNormal>::Ptr normals);
            void pcd_write(const std::string& filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudp);
            void estimate_normal(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudp);
            void average_normals(const pcl::PointCloud<pcl::PointNormal>::Ptr normals);
            void show_aver_normals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudp, Eigen::Vector3f normals, Eigen::Vector4f centroid);


            // void run();


        public:     // public variable
            pcl::PointCloud<pcl::PointXYZRGB> cloud_;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p_;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p_pt_;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p_ds_;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p_plane_;
            pcl::PointCloud<pcl::PointNormal>::Ptr normals_;
            Eigen::Vector3f ave_normal_;

            float plane_area_;
            bool process_pointcloud_;

        private: 
            
            // void AuboAPICallback(const std_msgs::Float32MultiArray::ConstPtr &msg);

    };
}

#endif /* TARGET_PLANE_ */
