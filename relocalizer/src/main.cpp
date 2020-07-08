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

Eigen::Matrix4f EstimateCorrespondence(CFPFH &MovFPFH, CFPFH &RefFPFH, double dDistThr)
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

void points_callback(const sensor_msgs::PointCloud2ConstPtr &points_msg)
{
    ROS_INFO("points_callback ...");
    if (is_match)
        return;

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
    cout << "Tf0 " << Tf0 << endl;

    is_match = false;
    cout << "Finished match " << timecal.toc() << "ms" << endl;
}

unsigned short Float32toFloat16(float fValue)
{
    return static_cast<unsigned short>((fValue + 1024.f) / (4096.f / 0xFFFF));
}

float Float16toFloat32(unsigned short fFloat16)
{
    return fFloat16 * (4096.0f / 65535.0f) - 1024.f;
}

void test2()
{
    float a = 1.02;
    unsigned short  b = Float32toFloat16(a);
    float a1 = Float16toFloat32(b);
    std::cout << "b:" << b  << " a1:"<<a1<< std::endl;

    float aa = -1.02;
    unsigned short  bb = Float32toFloat16(aa);
    float aa1 = Float16toFloat32(bb);
    std::cout << "bb:" << bb << " aa1:"<<aa1<< std::endl;
}

void test()
{
    char buffer[4];
    int ret = snprintf(buffer, sizeof buffer, "%f", 1.5);
    if (ret > 0)
    {
        for (int i = 0; i < 4; i++)
        {
            printf("buffer %x \n", buffer[i]);
        }
        float vOut = (float)strtod(&(buffer[0]), NULL);
        ROS_INFO_STREAM("get tempture:" << vOut);

        float f = 0;
        char array[] = {buffer[0], buffer[1], buffer[2], buffer[3]};
        std::memcpy(&f, array, 4);
        printf("float: %f \n", f);
    }

    //deal data float -> 2byte
    for (int i = -100; i < 10; i++)
    {
        float f = i * 1.1111;
        std::cout << "data:" << f << std::endl;
        f = f + 100;
        //deal
        int in = (int)f;
        unsigned char inn = in > 255 ? 255 : in;
        int ff = (f - in) * 100;
        //std::cout << "ff:"<<ff << std::endl;
        unsigned char dcc = (unsigned char)ff;
        printf("inn=%x dcc=%x\n", inn, dcc);
        //std::cout << "inn:"<<inn << " dcc:" <<dcc << std::endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "relocalizer");
    ros::NodeHandle nh_;

    ROS_INFO("relocalizer node started...");
    ros::Subscriber points_sub = nh_.subscribe("/velodyne_points", 1, &points_callback);
    ros::Rate rate(10);

    //load map
    // std::string globalmap_pcd = nh_.param<std::string>("/relocalizer/map", "");
    // cout << "map:" << globalmap_pcd << endl;

    // globalmap.reset(new pcl::PointCloud<pcl::PointXYZ>());
    // pcl::io::loadPCDFile(globalmap_pcd, *globalmap);

    // timecal.tic();

    // mapFPFH.setInputCloud(globalmap);
    // mapFPFH.compute();

    test2();
    cout << "Finished CFPFH Initial " << timecal.toc() << "ms" << endl;

    while (ros::ok())
    {
        cout << "loop ..." << endl;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}