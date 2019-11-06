/***************************************************************************
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

#ifndef SAWYER_GRAVITY_CONTROLLER_H
#define SAWYER_GRAVITY_CONTROLLER_H

#include <sawyer_sim_controllers/joint_array_controller.h>
#include <intera_core_msgs/SEAJointState.h>
#include <std_msgs/Empty.h>
#include <realtime_tools/realtime_box.h>
#include <sawyer_sim_controllers/sawyer_joint_effort_controller.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <control_toolbox/pid.h>

namespace sawyer_sim_controllers
{
  class SawyerGravityController : public sawyer_sim_controllers::JointArrayController<sawyer_effort_controllers::JointEffortController>
  {
  public:
    SawyerGravityController() : sawyer_sim_controllers::JointArrayController<sawyer_effort_controllers::JointEffortController>() { };

    virtual ~SawyerGravityController() {sub_joint_command_.shutdown();}
    virtual bool init(sawyer_hardware_interface::SharedJointInterface* hw, ros::NodeHandle &n) override;
    void setCommands();

  private:
    ros::Subscriber sub_joint_command_;
    ros::Subscriber sub_gravity_disable_;
    realtime_tools::RealtimeBox< std::shared_ptr<const ros::Time > > box_disable_time_;
    ros::Duration gravity_disable_timeout_;

    void gravityCommandCB(const intera_core_msgs::SEAJointStateConstPtr& msg);
    void gravityDisableCB(const std_msgs::Empty& msg);
  };
}

#endif
