// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#include "svo_ros/svo_interface.h"
#include <ros/callback_queue.h>
#include <vikit/params_helper.h>

namespace svo {

SvoInterface::SvoInterface( const ros::NodeHandle& nh_,
                            const ros::NodeHandle& pnh_) :
  vo_(NULL),
  publish_markers_(vk::getParam<bool>("svo/publish_markers", true)),
  publish_dense_input_(vk::getParam<bool>("svo/publish_dense_input", false)),
  remote_input_(""),
  cam_(NULL),
  quit_(false),
  pnh(pnh_),
  nh(nh_)
{
  // Create Camera
  if(!vk::camera_loader::loadFromRosNs("svo", cam_))
    throw std::runtime_error("Camera model not correctly specified.");

  // Get initial position and orientation
  visualizer_.T_world_from_vision_ = Sophus::SE3(
      vk::rpy2dcm(Vector3d(vk::getParam<double>("svo/init_rx", 0.0),
                           vk::getParam<double>("svo/init_ry", 0.0),
                           vk::getParam<double>("svo/init_rz", 0.0))),
      Eigen::Vector3d(vk::getParam<double>("svo/init_tx", 0.0),
                      vk::getParam<double>("svo/init_ty", 0.0),
                      vk::getParam<double>("svo/init_tz", 0.0)));

  // Init VO and start
  vo_ = new svo::FrameHandlerMono(cam_);
  vo_->start();
}

SvoInterface::~SvoInterface()
{
  delete vo_;
  delete cam_;
  if(user_input_thread_ != NULL)
    user_input_thread_->stop();
  if (image_thread)
      image_thread->join();
}

void SvoInterface::subscribeImage()
{
    image_thread = std::unique_ptr<std::thread>(new std::thread(&SvoInterface::imageLoop, this));
}

void SvoInterface::imageLoop()
{
    SVO_INFO_STREAM("SvoNode: Started Image loop.");

    ros::NodeHandle nh;
    ros::CallbackQueue queue;
    nh.setCallbackQueue(&queue);
    image_transport::ImageTransport it(nh);
    std::string cam_topic = vk::getParam<std::string>("svo/cam_topic", "camera/image_raw");
    image_transport::Subscriber it_sub = it.subscribe(cam_topic, 5, &svo::SvoInterface::imgCb, this);

    while(ros::ok() && !quit_)
    {
        queue.callAvailable(ros::WallDuration(0.1));
    }
}

void SvoInterface::subscribeUserActions()
{
    if(vk::getParam<bool>("svo/accept_console_user_input", true))
      user_input_thread_ = std::make_shared<vk::UserInputThread>();

    std::string remote_key_topic = vk::getParam<std::string>("remote_key_topic", "svo/remote_key");
      sub_remote_key_ = nh.subscribe(remote_key_topic, 5, &svo::SvoInterface::remoteKeyCb, this);
}

void SvoInterface::imgCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat img;
  try {
    img = cv_bridge::toCvShare(msg, "mono8")->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  processUserActions();
  vo_->addImage(img, msg->header.stamp.toSec());
  visualizer_.publishMinimal(img, vo_->lastFrame(), *vo_, msg->header.stamp.toSec());

  if(publish_markers_ && vo_->stage() != FrameHandlerBase::STAGE_PAUSED)
    visualizer_.visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());

  if(publish_dense_input_)
    visualizer_.exportToDense(vo_->lastFrame());

  if(vo_->stage() == FrameHandlerMono::STAGE_PAUSED)
    usleep(100000);
}

void SvoInterface::processUserActions()
{
  char input = remote_input_.c_str()[0];
  remote_input_ = "";

  if(user_input_thread_ != NULL)
  {
    char console_input = user_input_thread_->getInput();
    if(console_input != 0)
      input = console_input;
  }

  switch(input)
  {
    case 'q':
      quit_ = true;
      printf("SVO user input: QUIT\n");
      break;
    case 'r':
      vo_->reset();
      printf("SVO user input: RESET\n");
      break;
    case 's':
      vo_->start();
      printf("SVO user input: START\n");
      break;
    default: ;
  }
}

void SvoInterface::remoteKeyCb(const std_msgs::StringConstPtr& key_input)
{
  remote_input_ = key_input->data;
}

} // namespace svo
