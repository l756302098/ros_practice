#include "ros/ros.h"
#include "std_msgs/String.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <sstream>
#include <iostream>

using PointT = pcl::PointXYZI;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "map_server");

  ros::NodeHandle nh;

  pcl::PointCloud<PointT>::Ptr globalmap;
  ros::Publisher globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/pcd/globalmap", 1, true);
  std::string globalmap_pcd = nh.param<std::string>("/map_server/globalmap_pcd", "");
  // load global map from PCD file
  globalmap.reset(new pcl::PointCloud<PointT>());
  pcl::io::loadPCDFile(globalmap_pcd, *globalmap);
  globalmap->header.frame_id = "map";

  // downsample globalmap
  double downsample_resolution = nh.param<double>("/map_server/downsample_resolution", 0.1);
  boost::shared_ptr<pcl::ApproximateVoxelGrid<PointT>> voxelgrid(new pcl::ApproximateVoxelGrid<PointT>());
  voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
  voxelgrid->setInputCloud(globalmap);

  pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
  voxelgrid->filter(*filtered);
  globalmap = filtered;
  globalmap_pub.publish(globalmap);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}