#include <ros/package.h>
#include <string>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/config.h>
#include <svo_ros/visualizer.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <thread>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <vikit/abstract_camera.h>
#include <vikit/camera_loader.h>
#include <vikit/user_input_thread.h>
#include <ros/ros.h>

namespace svo {

/// SVO Interface
class SvoInterface
{
public:
  svo::FrameHandlerMono* vo_;
  svo::Visualizer visualizer_;
  bool publish_markers_;                 //!< publish only the minimal amount of info (choice for embedded devices)
  bool publish_dense_input_;
  std::shared_ptr<vk::UserInputThread> user_input_thread_;
  std::unique_ptr<std::thread> image_thread;

  std::string remote_input_;
  vk::AbstractCamera* cam_;
  bool quit_;

  SvoInterface( const ros::NodeHandle& nh_,
                const ros::NodeHandle& pnh_);
  virtual ~SvoInterface();

  void imgCb(const sensor_msgs::ImageConstPtr& msg);
  void processUserActions();
  void remoteKeyCb(const std_msgs::StringConstPtr& key_input);

  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  void subscribeImage();
  void subscribeUserActions();
  void imageLoop();

  ros::Subscriber sub_remote_key_;
};

}
