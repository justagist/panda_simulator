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

#ifndef SAWYER_HEAD_CONTROLLER_H_
#define SAWYER_HEAD_CONTROLLER_H_

#include <ros/node_handle.h>
#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_buffer.h>
#include <intera_core_msgs/HeadPanCommand.h>
#include <sawyer_sim_controllers/sawyer_joint_position_controller.h>  // used for controlling individual joints
#include <sawyer_hardware_interface/shared_joint_interface.h>

namespace sawyer_sim_controllers
{
class SawyerHeadController : public controller_interface::Controller<sawyer_hardware_interface::SharedJointInterface>
{
public:
  SawyerHeadController();
  ~SawyerHeadController();

  bool init(sawyer_hardware_interface::SharedJointInterface* robot, ros::NodeHandle& n);
  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);
  void updateCommands();

private:
  ros::NodeHandle nh_;
  /** Last commanded position. */
  class HeadCommand{
    public:
      double position;
      double velocity;
  };
  realtime_tools::RealtimeBuffer< std::shared_ptr<HeadCommand> > head_command_buffer;

  static const double HEAD_PAN_DEADBAND_RADIANS;  // in radians
  static const double HEAD_PAN_DEADBAND_VELOCITY; // in radians per second
  ros::Publisher head_state_pub_;
  ros::Timer head_state_pub_timer_;
  void publishHeadState(const ros::TimerEvent& event);

  size_t n_joints;
  std::string topic_name;

  bool new_command;  // true when an unproccessed new command is in the realtime buffer
  size_t update_counter;

  // Command subscriber
  ros::Subscriber head_command_sub;

  /**
   * @brief Callback from a recieved goal from the published topic message
   * @param msg trajectory goal
   */
  void commandCB(const intera_core_msgs::HeadPanCommandConstPtr& msg);

  // Create an effort-based joint position controller for every joint
  std::shared_ptr<sawyer_effort_controllers::JointPositionController> head_controller_ptr;
};

}  // namespace

#endif  // SAWYER_HEAD_CONTROLLER_H_
