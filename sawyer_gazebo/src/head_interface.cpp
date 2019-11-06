/***************************************************************************
* Copyright (c) 2018, Rethink Robotics Inc.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
**************************************************************************/

#include <sawyer_gazebo/head_interface.h>

#include <intera_core_msgs/HeadState.h>
#include <controller_manager_msgs/SwitchController.h>

//ROS-Opencv Headers
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

namespace sawyer_gazebo
{

const double HeadInterface::IMG_LOAD_ON_STARTUP_DELAY = 20;

HeadInterface::~HeadInterface()
{
  if (thread_head_display_ && thread_head_display_->joinable())
  {
    thread_head_display_->join();
  }
}

bool HeadInterface::init(ros::NodeHandle& nh,
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager)
{
  // As this is part of the Gazebo process, the initialize function cannot block
  // without preventing Gazebo from starting up. So, if there is a need to do
  // something immediately after startup (like publish an image), it needs to
  // be done in a thread.
  thread_head_display_ = std::make_unique<std::thread>(&HeadInterface::startHead,
                                                       this, std::ref(nh), controller_manager);
  return true;
}

bool HeadInterface::startHead(const ros::NodeHandle& nh, boost::shared_ptr<controller_manager::ControllerManager> controller_manager)
{
  bool ret = true;
  std::string img_path;
  if (!nh.getParam("/img_path_head_display", img_path))
  {
    ROS_ERROR_NAMED("head_display",
        "Could not load img_path_head_display path parameter server: %s",
        img_path.c_str());
    ret = false;
  }
  else
  {
    image_transport::ImageTransport img_trans(nh);
    image_transport::Publisher display_pub = img_trans.advertise("head_display", 1);
    // Read OpenCV Mat image and convert it to ROS message
    auto cv_ptr = std::make_unique<cv_bridge::CvImage>();
    try
    {
      cv_ptr->image = cv::imread(img_path, CV_LOAD_IMAGE_UNCHANGED);
      if (cv_ptr->image.data)
      {
        cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
        // Wait for the VideoPlugin screen to load, or timeout after delay
        ret = waitForSubscriber(display_pub, IMG_LOAD_ON_STARTUP_DELAY, 2);
        display_pub.publish(cv_ptr->toImageMsg());
      }
    }
    catch (std::exception e)
    {
      ROS_WARN_NAMED("head_display",
        "Unable to load the Startup picture on Sawyer display screen %s", e.what());
      ret = false;
    }
  }
  // Start Head Controller
  std::vector < std::string > start_controllers;
  std::vector < std::string > stop_controllers;
  start_controllers.push_back("head_position_controller");
  if (!controller_manager->switchController(start_controllers, stop_controllers,
                         controller_manager_msgs::SwitchController::Request::BEST_EFFORT))
  {
    ROS_ERROR_STREAM_NAMED("ros_control_plugin", "Failed to start head controller");
    ret = false;
  }
  return ret;
}

bool HeadInterface::waitForSubscriber(const image_transport::Publisher& pub, const double wait_time,
                                      const std::size_t num_req_sub)
{
  // Will wait at most this amount of time
  ros::Time max_time(ros::Time::now() + ros::Duration(wait_time));
  auto zero_time = std::numeric_limits<double>::epsilon();
  // How often to check for subscribers
  ros::Rate poll_rate(50);
  // This returns only the number of subscribers that have already
  // established their direct connections to this publisher
  int num_existing_subscribers = pub.getNumSubscribers();
  if (wait_time > zero_time && num_existing_subscribers == 0)
  {
    ROS_INFO_STREAM_NAMED("head_display", "Topic '" << pub.getTopic() << "' waiting for subscriber...");
  }
  // Wait for subscriber
  while (num_existing_subscribers < num_req_sub)
  {
    ROS_DEBUG_STREAM_THROTTLE_NAMED(2.0, "head_display", "Waiting " << max_time - ros::Time::now()
                                    << " num_existing_sub "<< num_existing_subscribers
                                    << " of required " << num_req_sub);
    if (wait_time < zero_time || ros::Time::now() > max_time || !ros::ok())  // Check if timed out
    {
      ROS_WARN_STREAM_NAMED("head_display", "Topic '" << pub.getTopic() << "' unable to connect to " << num_req_sub << " subscribers within "
                            << wait_time << " sec. It is possible initially published visual messages will be lost.");
      return false;
    }
    ros::spinOnce();
    poll_rate.sleep();
    num_existing_subscribers = pub.getNumSubscribers();
  }
  return true;
}

}  // namespace sawyer_gazebo
