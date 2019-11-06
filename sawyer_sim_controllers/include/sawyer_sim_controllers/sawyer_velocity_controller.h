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

#ifndef SAWYER_VELOCITY_CONTROLLER_H
#define SAWYER_VELOCITY_CONTROLLER_H

#include <sawyer_sim_controllers/joint_array_controller.h>
#include <intera_core_msgs/JointCommand.h>
#include <sawyer_sim_controllers/sawyer_joint_velocity_controller.h>
#include <ros/node_handle.h>

#include <control_toolbox/pid.h>
#include <sawyer_hardware_interface/shared_joint_interface.h>

namespace sawyer_sim_controllers
{
  class SawyerVelocityController : public sawyer_sim_controllers::JointArrayController<sawyer_effort_controllers::JointVelocityController>
  {
  public:
    virtual ~SawyerVelocityController() {sub_joint_command_.shutdown();}
    virtual bool init(sawyer_hardware_interface::SharedJointInterface* hw, ros::NodeHandle &n);
    void setCommands();

  private:
    ros::Subscriber sub_joint_command_;

    void jointCommandCB(const intera_core_msgs::JointCommandConstPtr& msg);
  };
}

#endif
