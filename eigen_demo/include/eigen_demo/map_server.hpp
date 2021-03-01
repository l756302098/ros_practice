/*
 * @Descripttion: 
 * @version: 
 * @Author: li
 * @Date: 2021-02-28 13:02:00
 * @LastEditors: li
 * @LastEditTime: 2021-02-28 13:22:57
 */
#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

using PointT = pcl::PointXYZI;
class map_server
{
private:
    /* data */
    ros::NodeHandle nh;
    ros::Publisher globalmap_pub;
    pcl::PointCloud<PointT>::Ptr globalmap;
public:
    map_server(const ros::NodeHandle &nh_ = ros::NodeHandle("~")):nh(nh_){
        std::string globalmap_pcd = nh.param<std::string>("globalmap_pcd", "");
        globalmap.reset(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(globalmap_pcd, *globalmap);
        globalmap->header.frame_id = "map";
        globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 5, true);
        // downsample globalmap
        double downsample_resolution = nh.param<double>("downsample_resolution", 0.1);
        boost::shared_ptr<pcl::ApproximateVoxelGrid<PointT>> voxelgrid(new pcl::ApproximateVoxelGrid<PointT>());
        voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
        voxelgrid->setInputCloud(globalmap);

        pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
        voxelgrid->filter(*filtered);
        globalmap = filtered;

        globalmap_pub.publish(globalmap);
    }
    ~map_server();
};

map_server::~map_server()
{
}