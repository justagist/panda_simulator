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

#include <sawyer_sim_controllers/sawyer_velocity_controller.h>
#include <pluginlib/class_list_macros.h>

namespace sawyer_sim_controllers {
  bool SawyerVelocityController::init(sawyer_hardware_interface::SharedJointInterface* hw, ros::NodeHandle &n){
    if(!sawyer_sim_controllers::JointArrayController<sawyer_effort_controllers::JointVelocityController>::init(hw, n)) {
      return false;
    } else {
      std::string topic_name;
      if (n.getParam("topic", topic_name)) {
        ros::NodeHandle nh("~");
        sub_joint_command_ = nh.subscribe(topic_name, 1, &SawyerVelocityController::jointCommandCB, this);
      } else {
        sub_joint_command_ = n.subscribe("joint_command", 1, &SawyerVelocityController::jointCommandCB, this);
      }
    }
    return true;
  }

  void SawyerVelocityController::jointCommandCB(const intera_core_msgs::JointCommandConstPtr& msg) {
    if (msg->mode == intera_core_msgs::JointCommand::VELOCITY_MODE) {
      std::vector<Command> commands;

      if (msg->names.size() != msg->velocity.size()) {
        ROS_ERROR_STREAM_NAMED(JOINT_ARRAY_CONTROLLER_NAME, "Velocity commands size does not match joints size");
      }

      for (int i = 0; i < msg->names.size(); i++) {
        Command cmd = Command();
        cmd.name_ = msg->names[i];
        cmd.velocity_ = msg->velocity[i];
        commands.push_back(cmd);
      }
      command_buffer_.writeFromNonRT(commands);
      new_command_ = true;
    }
  }


  void SawyerVelocityController::setCommands() {
    // set the new commands for each controller
    std::vector<Command> command = *(command_buffer_.readFromRT());
    for (auto it = command.begin(); it != command.end(); it++) {
      controllers_[it->name_]->setCommand(it->velocity_);
    }
  }
}

PLUGINLIB_EXPORT_CLASS(sawyer_sim_controllers::SawyerVelocityController, controller_interface::ControllerBase)
