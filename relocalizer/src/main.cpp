#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/console/time.h>

#include <sensor_msgs/PointCloud2.h>

#include "CFPFH.cpp"

pcl::console::TicToc timecal;

pcl::PointCloud<pcl::PointXYZ>::Ptr globalmap;

using namespace std;

CFPFH mapFPFH; 
 bool is_match;

Eigen::Matrix4f EstimateCorrespondence(CFPFH& MovFPFH, CFPFH& RefFPFH, double dDistThr)
{
	pcl::registration::CorrespondenceEstimation<FEATURE_TYPE, FEATURE_TYPE> est;
	est.setInputSource(MovFPFH.m_cloud_feature);
	est.setInputTarget(RefFPFH.m_cloud_feature);
	pcl::CorrespondencesPtr corres(new pcl::Correspondences);
	est.determineCorrespondences(*corres);

	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejector_sac;
	rejector_sac.setInputSource(MovFPFH.m_cloud_keypoints);
	rejector_sac.setInputTarget(RefFPFH.m_cloud_keypoints);
	rejector_sac.setInlierThreshold(dDistThr); // the unit of distance is meter, not the squared distance, original is 2.5
	rejector_sac.setMaximumIterations(10000);  // original is 1000000
	rejector_sac.setRefineModel(true);
	rejector_sac.setInputCorrespondences(corres);
	pcl::CorrespondencesPtr correspondences_filtered(new pcl::Correspondences);
	rejector_sac.getCorrespondences(*correspondences_filtered);
	corres.swap(correspondences_filtered);
	//sprintf("Determine correspondences via ransac: %03d vs %03d\n",
	//corres->size(), correspondences_filtered->size());
	//// refined results.
	Eigen::Matrix4f Tf0 = Eigen::Matrix4f::Identity();
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> transformation_estimation;
	transformation_estimation.estimateRigidTransformation(*MovFPFH.m_cloud_keypoints, *RefFPFH.m_cloud_keypoints,
		*corres, Tf0);

	return Tf0;
}

void points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg) 
{
    ROS_INFO("points_callback ...");
    if(is_match) return;

    ROS_INFO("start match ...");
    is_match = true;
    //sensor_msgs::PointCloud2ConstPtr points_msg = points_msg;
    bool valid_match = false;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*points_msg, *cloud);

    timecal.tic();
    CFPFH RefFPFH;
	RefFPFH.setInputCloud(cloud);
	RefFPFH.compute();

    double dDistThr = 1.0; 
	Eigen::Matrix4f Tf0 = EstimateCorrespondence(mapFPFH, RefFPFH, dDistThr); 
    cout<<"Tf0 "<< Tf0 <<endl;

    is_match = false;
    cout<<"Finished match "<<timecal.toc()<<"ms"<<endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "relocalizer");
    ros::NodeHandle nh_;

    ROS_INFO("relocalizer node started...");
    ros::Subscriber points_sub = nh_.subscribe("/velodyne_points", 1, &points_callback);
    ros::Rate rate(10);

    //load map
    std::string globalmap_pcd = nh_.param<std::string>("/relocalizer/map", "");
    cout << "map:" << globalmap_pcd << endl;

    globalmap.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::io::loadPCDFile(globalmap_pcd, *globalmap);

    timecal.tic();
    
	mapFPFH.setInputCloud(globalmap); 
	mapFPFH.compute(); 
    cout<<"Finished CFPFH Initial "<<timecal.toc()<<"ms"<<endl;

    while (ros::ok())
    {
        cout<<"loop ..."<<endl;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}