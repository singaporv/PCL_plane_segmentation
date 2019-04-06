// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <iostream>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <visualization_msgs/Marker.h>
using namespace std;

 // Defining Output clouds
pcl::PCLPointCloud2::Ptr output_voxel_cloud (new pcl::PCLPointCloud2 ());
pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud_complete (new pcl::PointCloud<pcl::PointXYZ>) ;
pcl::PCLPointCloud2::Ptr output_total_cloud (new pcl::PCLPointCloud2 ());
pcl::PCLPointCloud2::Ptr output_plane1 (new pcl::PCLPointCloud2 ());
pcl::PCLPointCloud2::Ptr output_plane2 (new pcl::PCLPointCloud2 ());
pcl::PCLPointCloud2::Ptr output_plane3 (new pcl::PCLPointCloud2 ());
pcl::PCLPointCloud2::Ptr output_cube (new pcl::PCLPointCloud2 ());
pcl::PCLPointCloud2::Ptr output_plane1a (new pcl::PCLPointCloud2 ());
pcl::PCLPointCloud2::Ptr output_plane2a (new pcl::PCLPointCloud2 ());
pcl::PCLPointCloud2::Ptr output_plane3a (new pcl::PCLPointCloud2 ());

// Defining publishers
ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pub4;
ros::Publisher pub5;
ros::Publisher cubePub;
ros::Publisher pub3a;
ros::Publisher pub4a;
ros::Publisher pub5a;
ros::Publisher vis_pub;

  pcl::PointCloud<pcl::PointXYZ>::Ptr ExtractPlaneCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud, bool check = false){
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  //Create segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.005); // initial threhold 

  // Create filtering object
  seg.setInputCloud(xyz_cloud);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0){
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
  }
  std::cerr << "The new method is" << xyz_cloud -> width * xyz_cloud ->height  <<std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud_complete (new pcl::PointCloud<pcl::PointXYZ>) ;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(xyz_cloud);
  extract.setIndices(inliers);
  extract.setNegative(check);
  extract.filter(*xyz_cloud_complete);
  std::cerr << "Inside the function points: " << xyz_cloud_complete ->width * xyz_cloud_complete ->height << " data points." << std::endl;
 return xyz_cloud_complete;
}

Eigen::Vector3f findMean(pcl::PointCloud<pcl::PointXYZ>::Ptr plane)
{
  //find mean x, y, and z values from plane given by apoorv
  float avgx=0;
  float avgy=0;
  float avgz=0;
  for (int i =0; i<plane->points.size(); i++)
  {
    avgx += plane->points[i].x;
    avgy += plane->points[i].y;
    avgz += plane->points[i].z;

  }
  Eigen::Vector3f P(avgx/plane->points.size(),avgy/plane->points.size(),avgz/plane->points.size());
  
  return P;
}



void cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud)
{
  // Temporary cloud to store passthrough filtered cloud
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());


  // Voxel grid - To downsample the pointcloud for better speedd
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*output_voxel_cloud);
  
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setInputCloud(output_voxel_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 0.5);
  pass.filter (*cloud_filtered);

  pcl::fromPCLPointCloud2 (*cloud_filtered, *xyz_cloud);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  //Create segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.005); // threshold values (initial 0.01)

  // Filtering
  seg.setInputCloud(xyz_cloud);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0){
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
  }

  // Defining the point clouds for planes (plane1, plane2, plane3)
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane1 (new pcl::PointCloud<pcl::PointXYZ>) ;
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane2 (new pcl::PointCloud<pcl::PointXYZ>) ;
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane3 (new pcl::PointCloud<pcl::PointXYZ>) ;

  pcl::PointCloud<pcl::PointXYZ>::Ptr plane1a (new pcl::PointCloud<pcl::PointXYZ>) ;
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane2a (new pcl::PointCloud<pcl::PointXYZ>) ;
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane3a (new pcl::PointCloud<pcl::PointXYZ>) ;
  


  // Extracting planes (1 by 1) from complete cloud
  bool check = true;
  xyz_cloud_complete = ExtractPlaneCloud(xyz_cloud, check);
  plane1 = ExtractPlaneCloud(xyz_cloud_complete, true);
  plane2 = ExtractPlaneCloud(ExtractPlaneCloud(plane1, true), false);
  plane3 = ExtractPlaneCloud(ExtractPlaneCloud(xyz_cloud_complete, true), false);
  plane1 = ExtractPlaneCloud(xyz_cloud_complete, false);

  plane1a = plane1;
  plane2a = plane2;
  plane3a = plane3;


  //delete later
  for (size_t i = 0; i < plane1a->points.size (); ++i)
  {
    plane1a->points[i].x = plane1a->points[i].x + .02;
    plane1a->points[i].y = plane1a->points[i].y + .3;
    plane1a->points[i].z = plane1a->points[i].z + .00;
  }

  for (size_t i = 0; i < plane2a->points.size (); ++i)
  {
    plane2a->points[i].x = plane2a->points[i].x + .08;
    plane2a->points[i].y = plane2a->points[i].y + .0005;
    plane2a->points[i].z = plane2a->points[i].z + .00;
  }

  for (size_t i = 0; i < plane3a->points.size (); ++i)
  {
    plane3a->points[i].x = plane3a->points[i].x + .04;
    plane3a->points[i].y = plane3a->points[i].y + .07;
    plane3a->points[i].z = plane3a->points[i].z + .00;
  }
  //


  // Converting to pointcloud format
  pcl::toPCLPointCloud2 (*xyz_cloud_complete, *output_total_cloud);
  pcl::toPCLPointCloud2 (*plane1, *output_plane1);
  pcl::toPCLPointCloud2 (*plane2, *output_plane2);
  pcl::toPCLPointCloud2 (*plane3, *output_plane3);
  pcl::toPCLPointCloud2 (*plane1a, *output_plane1a);
  pcl::toPCLPointCloud2 (*plane2a, *output_plane2a);
  pcl::toPCLPointCloud2 (*plane3a, *output_plane3a);

  //get centroid
  Eigen::Vector3f center1 = findMean(plane1);
  Eigen::Vector3f center2 = findMean(plane2);
  Eigen::Vector3f center3 = findMean(plane3);
  std::cout<<(center1(0))<<std::endl;
  std::cout<<(center1(1))<<std::endl;
  std::cout<<(center1(2))<<std::endl;
  std::cout<<(center2(0))<<std::endl;
  std::cout<<(center2(1))<<std::endl;
  std::cout<<(center2(2))<<std::endl;
  std::cout<<(center3(0))<<std::endl;
  std::cout<<(center3(1))<<std::endl;
  std::cout<<(center3(2))<<std::endl;




// visuaization of te pointloud
  visualization_msgs::Marker marker;
  marker.header.frame_id = "camera_depth_frame";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = .44;
  marker.pose.position.y = 0.03;
  marker.pose.position.z = .02;
  marker.pose.orientation.x = 30;
  marker.pose.orientation.y = 50;
  marker.pose.orientation.z = -20;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = .12;
  marker.scale.y = .1;
  marker.scale.z = .05;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  //only if using a MESH_RESOURCE marker type:
 //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  vis_pub.publish( marker );





  // Publishing Voxels and full segmenataion of 3 planes and 3 individual planes
  pub1.publish (*output_voxel_cloud);
  pub2.publish (*output_total_cloud);
  pub3.publish (*output_plane1);
  pub4.publish (*output_plane2);
  pub5.publish (*output_plane3);
  cubePub.publish (*output_cube);

  pub3a.publish (*output_plane1a);
  pub4a.publish (*output_plane2a);
  pub5a.publish (*output_plane3a);
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "example");
  ros::NodeHandle nh;

  // Creating Subscriber
  ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 1, cloud_cb);

  // Creating Publisher
  pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/cloud/voxel_complete_set", 1);
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/cloud/segmentation_complete", 1);
  pub3 = nh.advertise<sensor_msgs::PointCloud2> ("/cloud/plane_1", 1);
  pub4 = nh.advertise<sensor_msgs::PointCloud2> ("/cloud/plane_2", 1);
  pub5 = nh.advertise<sensor_msgs::PointCloud2> ("/cloud/plane_3", 1);

  pub3a = nh.advertise<sensor_msgs::PointCloud2> ("/cloud/plane_1a", 1);
  pub4a = nh.advertise<sensor_msgs::PointCloud2> ("/cloud/plane_2a", 1);
  pub5a = nh.advertise<sensor_msgs::PointCloud2> ("/cloud/plane_3a", 1);

  cubePub = nh.advertise<sensor_msgs::PointCloud2> ("/cloud/cube", 1);
  vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );


  // Spin
  ros::spin ();
}












































































// #include <ros/ros.h>
// // PCL specific includes
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
// // #include <pcl/ros/conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>

// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/filters/voxel_grid.h>


// #include <pcl/ModelCoefficients.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/filters/extract_indices.h>

// ros::Publisher pub;

// Basic Point cloud
// void
// cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
// {
//   // Create a container for the data.
//   sensor_msgs::PointCloud2 output;

//   // Do data processing here...
//   output = *input;

//   // Publish the data.
//   pub.publish (output);
// }


// Voxel in point cloud
// void
// cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
// {
//   // Container for original & filtered data
//   pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
//   pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
//   pcl::PCLPointCloud2 cloud_filtered;

//   // Convert to PCL data type
//   pcl_conversions::toPCL(*cloud_msg, *cloud);

//   // Perform the actual filtering
//   pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//   sor.setInputCloud (cloudPtr);
//   sor.setLeafSize (0.1, 0.1, 0.1);
//   sor.filter (cloud_filtered);

//   // Convert to ROS data type
//   sensor_msgs::PointCloud2 output;
//   pcl_conversions::moveFromPCL(cloud_filtered, output);

//   // Publish the data
//   pub.publish (output);
// }



// Segmentation of point cloud --> returns pointcloud
// void
// cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
// {
//   // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
//   pcl::PointCloud<pcl::PointXYZ> cloud;
//   pcl::fromROSMsg (*input, cloud);

//   pcl::ModelCoefficients coefficients;
//   pcl::PointIndices inliers;
//   // Create the segmentation object
//   pcl::SACSegmentation<pcl::PointXYZ> seg;
//   // Optional
//   seg.setOptimizeCoefficients (true);
//   // Mandatory
//   seg.setModelType (pcl::SACMODEL_PLANE);
//   seg.setMethodType (pcl::SAC_RANSAC);
//   seg.setDistanceThreshold (0.01);

//   seg.setInputCloud (cloud.makeShared ());
//   seg.segment (inliers, coefficients);

//   // Publish the model coefficients
//   pcl_msgs::ModelCoefficients ros_coefficients;
//   pcl_conversions::fromPCL(coefficients, ros_coefficients);
//   pub.publish (ros_coefficients);
// }



// int
// main (int argc, char** argv)
// {
//   // Initialize ROS
//   ros::init (argc, argv, "my_pcl_tutorial");
//   ros::NodeHandle nh;

//   // Create a ROS subscriber for the input point cloud
//   ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 1, cloud_cb);

//   // Create a ROS publisher for the output model coefficients
//   // pub = nh.advertise<sensor_msgs::PointCloud2> ("point_cloud__voxel_out", 1);
//   pub = nh.advertise<pcl_msgs::ModelCoefficients> ("point_cloud_seg", 1);

//   // Spin
//   ros::spin ();
// }
