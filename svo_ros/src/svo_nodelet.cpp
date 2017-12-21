#include <ros/ros.h>
#include <svo_ros/svo_nodelet.h>
#include <svo_ros/svo_interface.h>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(svo::SvoNodelet, nodelet::Nodelet)

namespace svo {

SvoNodelet::~SvoNodelet()
{
  NODELET_INFO_STREAM("SVO quit");
  svo_interface->quit_ = true;
}

void SvoNodelet::onInit()
{
  ros::NodeHandle nh(getNodeHandle());
  ros::NodeHandle pnh(getPrivateNodeHandle());

  NODELET_INFO_STREAM("Initialized " <<  getName() << " nodelet.");
  svo_interface.reset(new SvoInterface(nh, pnh));
  svo_interface->subscribeImage();
  //svo_interface->subscribeUserActions();
}

} // namespace svo
