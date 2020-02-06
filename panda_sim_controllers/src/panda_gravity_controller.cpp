/***************************************************************************
* Adapted from sawyer_gravity_controller.cpp (sawyer_simulator package)

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

#include <panda_sim_controllers/panda_gravity_controller.h>
#include <pluginlib/class_list_macros.h>

namespace panda_sim_controllers {

  bool PandaGravityController::init(panda_hardware_interface::SharedJointInterface* hw, ros::NodeHandle &n){
    // TODO: use constant, don't hardcode ctrl_subtype ("GRAVITY_COMPENSATION")
    if(!panda_sim_controllers::JointArrayController<panda_effort_controllers::JointEffortController>::init(hw, n, "GRAVITY_COMPENSATION")) {
      return false;
    }

    sub_joint_command_ = n.subscribe("/panda_simulator/custom_franka_state_controller/robot_state", 1, &PandaGravityController::gravityCommandCB, this);

    if (!n.getParam("/robot_config/joint_names", joint_names_) ) {
      ROS_ERROR(
          "PandaPositionController: Invalid or no joint_names parameters provided, aborting "
          "controller init!");
      return false;
    }
    return true;
  }


  void PandaGravityController::gravityCommandCB(const franka_core_msgs::RobotStateConstPtr& msg) {

      std::vector<Command> commands;
      if (joint_names_.size() != msg->gravity.size()) {
        ROS_ERROR_STREAM_NAMED("PandaGravityController", "Gravity commands size does not match joints size");
      }
      for (int i = 0; i < msg->gravity.size(); i++) {
        Command cmd = Command();
        cmd.name_ = joint_names_[i];
        cmd.effort_ = msg->gravity[i];
        commands.push_back(cmd);
      }
      command_buffer_.writeFromNonRT(commands);
      new_command_ = true;
  }

  void PandaGravityController::setCommands() {
    // set the new commands for each controller
    std::vector<Command> command = *(command_buffer_.readFromRT());
    for (auto it = command.begin(); it != command.end(); it++) {
      controllers_[it->name_]->command_buffer_.writeFromNonRT(it->effort_);
    }
  }
}

PLUGINLIB_EXPORT_CLASS(panda_sim_controllers::PandaGravityController, controller_interface::ControllerBase)
