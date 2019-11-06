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

#include <sawyer_sim_controllers/sawyer_gravity_controller.h>
#include <pluginlib/class_list_macros.h>

namespace sawyer_sim_controllers {

  bool SawyerGravityController::init(sawyer_hardware_interface::SharedJointInterface* hw, ros::NodeHandle &n){
    // TODO: use constant, don't hardcode ctrl_subtype ("GRAVITY_COMPENSATION")
    if(!sawyer_sim_controllers::JointArrayController<sawyer_effort_controllers::JointEffortController>::init(hw, n, "GRAVITY_COMPENSATION")) {
      return false;
    }
    std::string command_topic;
    if (n.getParam("command_topic", command_topic)) {
      ros::NodeHandle nh("~");
      sub_joint_command_ = nh.subscribe(command_topic, 1, &SawyerGravityController::gravityCommandCB, this);
    } else {
      sub_joint_command_ = n.subscribe("gravity_command", 1, &SawyerGravityController::gravityCommandCB, this);
    }
    std::string disable_topic;
    if (n.getParam("disable_topic", disable_topic)) {
      ros::NodeHandle nh("~");
      sub_gravity_disable_ = nh.subscribe(disable_topic, 1, &SawyerGravityController::gravityDisableCB, this);
    } else {
      sub_gravity_disable_ = n.subscribe("gravity_disable", 1, &SawyerGravityController::gravityDisableCB, this);
    }
    // In order to disable the gravity compensation torques,
    // an empty message should be published at a frequency greater than (1/disable_timeout) Hz
    double disable_timeout;
    n.param<double>("disable_timeout", disable_timeout, 0.2);
    gravity_disable_timeout_ = ros::Duration(disable_timeout);
    return true;
  }

  void SawyerGravityController::gravityDisableCB(const std_msgs::Empty& msg) {
      auto p_disable_msg_time = std::make_shared<ros::Time>(ros::Time::now());
      box_disable_time_.set(p_disable_msg_time);
      ROS_INFO_STREAM_THROTTLE(60, "Gravity compensation torques are disabled...");
  }

  void SawyerGravityController::gravityCommandCB(const intera_core_msgs::SEAJointStateConstPtr& msg) {

      std::vector<Command> commands;
      if (msg->name.size() != msg->gravity_model_effort.size()) {
        ROS_ERROR_STREAM_NAMED(JOINT_ARRAY_CONTROLLER_NAME, "Gravity commands size does not match joints size");
      }
      std::shared_ptr<const ros::Time>  p_disable_msg_time;
      box_disable_time_.get(p_disable_msg_time);
      bool enable_gravity = !p_disable_msg_time || ((ros::Time::now() - *p_disable_msg_time.get()) > gravity_disable_timeout_);
      for (int i = 0; i < msg->name.size(); i++) {
        Command cmd = Command();
        cmd.name_ = msg->name[i];
        cmd.effort_ = enable_gravity ? msg->gravity_model_effort[i] : 0.0;
        commands.push_back(cmd);
      }
      command_buffer_.writeFromNonRT(commands);
      new_command_ = true;
  }

  void SawyerGravityController::setCommands() {
    // set the new commands for each controller
    std::vector<Command> command = *(command_buffer_.readFromRT());
    for (auto it = command.begin(); it != command.end(); it++) {
      controllers_[it->name_]->command_buffer_.writeFromNonRT(it->effort_);
    }
  }
}

PLUGINLIB_EXPORT_CLASS(sawyer_sim_controllers::SawyerGravityController, controller_interface::ControllerBase)
