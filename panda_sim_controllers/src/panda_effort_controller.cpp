/***************************************************************************
* Adapted from sawyer_effort_controller.cpp (sawyer_simulator package)

*
* @package: panda_sim_controllers
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

#include <panda_sim_controllers/panda_effort_controller.h>
#include <pluginlib/class_list_macros.h>

namespace panda_sim_controllers {
  bool PandaEffortController::init(panda_hardware_interface::SharedJointInterface* hw, ros::NodeHandle &n){
    if(!panda_sim_controllers::JointArrayController<panda_effort_controllers::JointEffortController>::init(hw, n)) {
      return false;
    } else {
      std::string topic_name;
      if (n.getParam("topic", topic_name)) {
        ros::NodeHandle nh("~");
        sub_joint_command_ = nh.subscribe(topic_name, 1, &PandaEffortController::jointCommandCB, this);
      } else {
        sub_joint_command_ = n.subscribe("joint_command", 1, &PandaEffortController::jointCommandCB, this);
      }
    }
    return true;
  }

  void PandaEffortController::jointCommandCB(const franka_core_msgs::JointCommandConstPtr& msg) {
    if (msg->mode == franka_core_msgs::JointCommand::TORQUE_MODE) {
      std::vector<Command> commands;

      if (msg->names.size() != msg->effort.size()) {
        ROS_ERROR_STREAM_NAMED(JOINT_ARRAY_CONTROLLER_NAME, "Effort commands size does not match joints size");
      }

      for (int i = 0; i < msg->names.size(); i++) {
        Command cmd = Command();
        cmd.name_ = msg->names[i];
        cmd.effort_ = msg->effort[i];
        commands.push_back(cmd);
      }
      command_buffer_.writeFromNonRT(commands);
      new_command_ = true;
    }
  }

  void PandaEffortController::setCommands() {
    // set the new commands for each controller
    std::vector<Command> command = *(command_buffer_.readFromRT());
    for (auto it = command.begin(); it != command.end(); it++) {
      controllers_[it->name_]->command_buffer_.writeFromNonRT(it->effort_);
    }
  }
}

PLUGINLIB_EXPORT_CLASS(panda_sim_controllers::PandaEffortController, controller_interface::ControllerBase)
