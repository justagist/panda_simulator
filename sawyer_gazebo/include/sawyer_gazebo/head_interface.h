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
#ifndef SAWYER_GAZEBO_HEAD_INTERFACE_H
#define SAWYER_GAZEBO_HEAD_INTERFACE_H

#include <string>
#include <thread>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <image_transport/image_transport.h>

namespace sawyer_gazebo
{
class HeadInterface
{
public:
  bool init(ros::NodeHandle& nh, boost::shared_ptr<controller_manager::ControllerManager> controller_manager);
  ~HeadInterface();

private:
  static const double IMG_LOAD_ON_STARTUP_DELAY;
  std::unique_ptr<std::thread> thread_head_display_;

  bool startHead(const ros::NodeHandle& nh, boost::shared_ptr<controller_manager::ControllerManager> controller_manager);

  /**
   * \brief Helper to wait until a ROS topic is available
   * \param publisher to wait for
   * \param amount of time to wait. if 0.0, will not block
   * \param number of subscribers to wait for
   * \return true on connection
   * \contribution davetcoleman
   */
  bool waitForSubscriber(const image_transport::Publisher& pub, const double wait_time,
                         const std::size_t num_req_sub = 2);

};
}  // namespace sawyer_gazebo
#endif  // SAWYER_GAZEBO_ARM_KINEMATICS_INTERFACE_H
