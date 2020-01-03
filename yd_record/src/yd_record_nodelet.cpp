#include "yd_record_nodelet.h"

namespace yd_record
{
YdRecordNodelet::YdRecordNodelet()
{
    ROS_INFO("SampleNodeletClass Constructor");
}

YdRecordNodelet::~YdRecordNodelet()
{
    ROS_INFO("SampleNodeletClass Destructor");
}

void YdRecordNodelet::onInit()
{
    NODELET_INFO("SampleNodeletClass - %s", __FUNCTION__);
}
} // namespace yd_record

PLUGINLIB_EXPORT_CLASS(yd_record::YdRecordNodelet, nodelet::Nodelet)