#include "targetPlane/target_plane.h"
#include "pcl_conversions/pcl_conversions.h"


namespace target_plane 
{

// constructor
targetPlane::targetPlane()
{
  // plane_area_ = 0.0;
  // process_pointcloud_ = true;

}

// destructor
// targetPlane::~targetPlane()
// {

// }
// get a frame of pointcloud from a pointcloud topic


void targetPlane::readFromFile(const std::string& filename)
{ 
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, cloud_)) //* load the file
  {
    PCL_ERROR ("Couldn't read the point cloud data file \n");
    //return (-1);
  }
  *cloud_p_ = cloud_;
  std::cout << "Loaded "
            << cloud_.width * cloud_.height
            << " data points from the pcd file"
            << std::endl;
}

// passThrough filter
void targetPlane::passthrough_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZRGB> distance_pass;
    distance_pass.setInputCloud (cloud);
    distance_pass.setFilterFieldName ("z");
    distance_pass.setFilterLimits (0.2, 1);
    //pass.setFilterLimitsNegative (true);
    distance_pass.filter (*cloud_p_pt_);
    std::cerr << "Cloud after passthrough filtering: " << cloud_p_pt_->size() << std::endl;
}

void targetPlane::downsampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudp)
{
    std::cerr << "PointCloud before downsampling: "<< cloudp->size() <<" data points." << std::endl;
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloudp);
    // define the size of the the leaf (in meter) = voxel size
    sor.setLeafSize (0.005f, 0.005f, 0.005f);
    // perform the downsampling filter then save the new data to cloud_filtered
    sor.filter (*cloud_p_ds_);
    std::cerr << "After downsampling: "<<cloudp->size()<<" data points." << std::endl;
}

void targetPlane::get_planes(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudp)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloudp, *cloud_p);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    // variable to store the point cloud belong to the desired object
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());  
    // Create the segmentation object -> filter the desired point belong to the object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true); //> set this function using refine optimize Coefficient
    // Mandatory - set parameter for the planar filter
    seg.setModelType (pcl::SACMODEL_PLANE); // -> segment the plane in the point cloud
    // -> RANSAC = Random Sample Consensus = estimator method
    seg.setMethodType (pcl::SAC_RANSAC);    //-> use 2 step to determine the outliers or inliers
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.01);    // -> tolerate range to define the inliers & outlier
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    int nr_points = (int) cloud_p->points.size ();
    int cloudsize = 0;
    
    // While 30% of the original cloud is still there
    while (cloud_p->points.size() > 0.3*nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_p);
        // seg.segment() result in the inliers point and coefficients
        seg.segment (*inliers, *coefficients); //save all points belong to the plane to inliners
        // outlier -> mau ngoai lai, out of observation range -> skew the estimation result
        // inlier -> sample lie inside the range -> fit with the model 
        if (inliers->indices.size () == 0) // -> if there is no inliers -> no point fit with the object
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        // input the pointcloud into Extract() function
        extract.setInputCloud (cloud_p_);
        // input a pointer to vector of indices (inliers)->represent the input data
        extract.setIndices(inliers);
        // set whether the indices should be returned, or all points_except_the indices
        extract.setNegative (false); //false -> return the indices
        // save the result to the cloud_p
        extract.filter (*cloud_p_plane_);
        // if no change in the indices -> stop
        if (cloudsize == cloud_p_plane_->width*cloud_p_plane_->height)
            break;
        std::cerr <<"PointCloud plane: "<< cloud_p_plane_->width*cloud_p_plane_->height <<" data points."<< std::endl;
        cloud_p.swap (cloud_p_plane_);
        cloudsize = cloud_p_plane_->width*cloud_p_plane_->height;
    }
}

void targetPlane::show_pcd(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudp, int clus_order, std::string& titlename)
{
    int v1(0);
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (titlename));
    viewer->setBackgroundColor (0, 0, 0);
    if (clus_order != 0)
    {   
        std::stringstream ss1;
        // convert clus_order to string
        ss1 << clus_order;
        
        std::string ss = "Cluster " + ss1.str();
        //viewer->addText("Cluster "+ ss2, 80, 50, "v1 text", v1);
        //addText (const std::string &text, int xpos, int ypos, int fontsize,double r, double g, double b, const std::string &id="",int viewport=0)
        viewer->addText(ss, 80, 50, 20, 120, 120, 200, "v1 text", v1);
    }   
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloudp, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloudp, single_color, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce (100);
    }
}

void targetPlane::show_centroid(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudp, Eigen::MatrixXf& centroid3D)
{
  // int v1(0);
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Showing Centroid"));
  viewer->setBackgroundColor (0, 0, 0);
  // Add text to the screen
  //viewer->addText("Radius: 0.002", 50, 50, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloudp, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloudp, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  int clusterNumber1 = centroid3D.rows();
  std::string id[clusterNumber1];
  pcl::ModelCoefficients line_coeff[clusterNumber1]; //, line_coeff2;

  for (int i=0; i < clusterNumber1; ++i)
  {
      line_coeff[i].values.resize (6);    // We need 6 values
      line_coeff[i].values[0] = 0; //point_on_line.x ();
      line_coeff[i].values[1] = 0;//point_on_line.y ();
      line_coeff[i].values[2] = 0; //point_on_line.z ();
      line_coeff[i].values[3] = centroid3D(i,0);//-0.078; //line_direction.x ();
      line_coeff[i].values[4] = centroid3D(i,1);//0.126; //line_direction.y ();
      line_coeff[i].values[5] = centroid3D(i,2);//0.49; //line_direction.z ();
  
      id[i] = "line"+i;
      viewer->addLine(line_coeff[i], id[i], 0);
  }
  while (!viewer->wasStopped())
  {
    viewer->spinOnce (100);
  }
}

void targetPlane::show_normals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudp, const pcl::PointCloud<pcl::PointNormal>::Ptr normals)
{
    // int v1(0);
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Showing Normals"));
    viewer->setBackgroundColor (0, 0, 0);
    // Add text to the screen
    //viewer->addText("Radius: 0.002", 50, 50, "v1 text", v1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloudp, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloudp, single_color, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (0.4);
    //Para: cloud   the input point cloud dataset containing the XYZ data
      //normals   the input point cloud dataset containing the normal data
      //level = 1   display only every level'th point (default: 100)
      // scale = 0.2  the normal arrow scale (default: 0.02m)
      //id = "normals"  the point cloud object id (default: cloud) 
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::PointNormal>(cloudp, normals, 1, 0.04, "normals"); 
  while (!viewer->wasStopped())
  {
    viewer->spinOnce (100);
  }
}


void targetPlane::pcd_write(const std::string& filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudp)
{  
  pcl::PCDWriter writer;
  std::stringstream ss;
  ss << "prep_"+filename+".pcd";
  writer.write<pcl::PointXYZRGB> (ss.str(), *cloudp, false); //*
  std::cout<< "Saved "<< cloudp->points.size()<< " data points to file: "<< ss.str() << std::endl;
}

// estimate the normal vector 
void targetPlane::estimate_normal(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudp)
{
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal> ne;
  ne.setInputCloud (cloudp);
  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  tree->setInputCloud(cloudp);
  ne.setSearchMethod (tree);
  ne.setViewPoint (std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  // pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.02);
  // Compute the features
  ne.compute(*normals_);
}

// normals vector of a surface
void targetPlane::average_normals(const pcl::PointCloud<pcl::PointNormal>::Ptr normals)
{
  float sum_x=0, sum_y=0, sum_z=0;
  for (int i=0; i<normals->size(); i++)
  {
    sum_x += normals->points[i].normal_x;

    sum_y += normals->points[i].normal_y;
    sum_z += normals->points[i].normal_z;
  }
  // Eigen::Vector3f aver_nor;
  ave_normal_[0] = sum_x/(normals->size());
  ave_normal_[1] = sum_y/(normals->size());
  ave_normal_[2] = sum_z/(normals->size());
  // return aver_nor;
}

void targetPlane::show_aver_normals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudp, Eigen::Vector3f normals, Eigen::Vector4f centroid)
{
  // int v1(0);
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Showing Average Normals"));
  viewer->setBackgroundColor (0, 0, 0);
  // Add text to the screen
  //viewer->addText("Radius: 0.002", 50, 50, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloudp, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloudp, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (0.4);
  
  pcl::ModelCoefficients line_coeff;
  
  line_coeff.values.resize (6);    // We need 6 values
  line_coeff.values[0] = centroid[0];
  line_coeff.values[1] = centroid[1];
  line_coeff.values[2] = centroid[2]; 
  line_coeff.values[3] = centroid[0]+normals[0];
  line_coeff.values[4] = centroid[1]+normals[1];
  line_coeff.values[5] = centroid[2]+normals[2];

  std::string id = "normal vector";
  viewer->addLine(line_coeff, id, 0);

  while (!viewer->wasStopped())
  {
    viewer->spinOnce (100);
    //std::this_thread::sleep_for(100ms);
  }
  // return (viewer);
}

void targetPlane::calculateAreaPolygon(const pcl::PointCloud<pcl::PointXYZRGB> &polygon)
{
  float area=0.0;
  int num_points = polygon.size();
  int j = 0;
  Eigen::Vector3f va,vb,res;
  res(0) = res(1) = res(2) = 0.0f;
  for (int i = 0; i < num_points; ++i)
  {
    j = (i + 1) % num_points;
    // convert a point to a vector form
    va = polygon[i].getVector3fMap();
    vb = polygon[j].getVector3fMap();
    res += va.cross(vb);
  }
  // calculate the parallelogram
  area=res.norm();
  plane_area_ = 0.5*res.norm();
}

void targetPlane::pointcloud_Callback(const sensor_msgs::PointCloud2& msg)
{
  if (process_pointcloud_)
  {
    pcl::fromROSMsg(msg, *cloud_p_);
    ROS_INFO("Got point cloud with %ld points", cloud_p_->size());
    targetPlane::passthrough_filter(cloud_p_);

    process_pointcloud_ = false;
  }
  else
    ROS_INFO("Not process the incoming point cloud"); 
}

}   // end of namespace