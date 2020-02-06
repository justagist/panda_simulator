/***************************************************************************
* Adapted from arm_controller_interface.h (sawyer_simulator package)

*
* @package: panda_gazebo
* @metapackage: panda_simulator
* @author: Saif Sidhik <sxs1412@bham.ac.uk>
*

**************************************************************************/

/***************************************************************************
* Copyright (c) 2019-2020, Saif Sidhik
* Copyright (c) 2013-2018, Rethink Robotics Inc.
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
#ifndef _PANDA_GAZEBO___ARM_CONTROLLER_INTERFACE_H_
#define _PANDA_GAZEBO___ARM_CONTROLLER_INTERFACE_H_

#include <mutex>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <std_msgs/Float64.h>
#include <realtime_tools/realtime_box.h>
#include <franka_core_msgs/JointCommand.h>


namespace panda_gazebo {
  class ArmControllerInterface
  {
  public:
    void init(ros::NodeHandle& nh,
         boost::shared_ptr<controller_manager::ControllerManager> controller_manager);

  private:
    // mutex for re-entrant calls to modeCommandCallback
    std::mutex mtx_;
    int current_mode_;
    std::string side_;

    realtime_tools::RealtimeBox< std::shared_ptr<const ros::Duration > > box_timeout_length_;
    realtime_tools::RealtimeBox< std::shared_ptr<const ros::Time > > box_cmd_timeout_;

    ros::Timer cmd_timeout_timer_;

    ros::Subscriber joint_command_timeout_sub_;
    ros::Subscriber joint_command_sub_;
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  protected:
    void jointCommandTimeoutCallback(const std_msgs::Float64 msg);
    void jointCommandCallback(const franka_core_msgs::JointCommandConstPtr& msg);
    std::string getControllerString(std::string mode_str);
    bool switchControllers(int control_mode);
    void commandTimeoutCheck(const ros::TimerEvent& e);


  };
}
#endif // #ifndef __PANDA_GAZEBO__ARM_CONTROLLER_INTERFACE_H_
