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

#include <sawyer_sim_controllers/sawyer_position_controller.h>
#include <pluginlib/class_list_macros.h>

namespace sawyer_sim_controllers {
  bool SawyerPositionController::init(sawyer_hardware_interface::SharedJointInterface* hw, ros::NodeHandle &n){
    if(!sawyer_sim_controllers::JointArrayController<sawyer_effort_controllers::JointPositionController>::init(hw, n)) {
      return false;
    } else {
      std::string topic_name;
      if (n.getParam("topic_joint_command", topic_name)) {
        ros::NodeHandle nh("~");
        sub_joint_command_ = nh.subscribe(topic_name, 1, &SawyerPositionController::jointCommandCB, this);
      } else {
        sub_joint_command_ = n.subscribe("joint_command", 1, &SawyerPositionController::jointCommandCB, this);
      }
      std::string topic_speed_ratio;
      if (n.getParam("topic_set_speed_ratio", topic_speed_ratio)) {
        ros::NodeHandle nh("~");
        sub_speed_ratio_ = nh.subscribe(topic_speed_ratio, 1, &SawyerPositionController::speedRatioCallback, this);
      } else {
        sub_speed_ratio_ = n.subscribe("set_speed_ratio", 1, &SawyerPositionController::speedRatioCallback, this);
      }
      std::shared_ptr<std_msgs::Float64> speed_ratio(new std_msgs::Float64());
      speed_ratio->data = 0.3; // Default to 30% max urdf speed
      speed_ratio_buffer_.set(speed_ratio);
    }
    return true;
  }

  void SawyerPositionController::speedRatioCallback(const std_msgs::Float64 msg) {
    std::shared_ptr<std_msgs::Float64> speed_ratio(new std_msgs::Float64());
    if(msg.data > 1){
      speed_ratio->data = 1;
    }
    else if(msg.data < 0){
      speed_ratio->data = 0;
    }
    else{
      speed_ratio->data = msg.data;
    }
    speed_ratio_buffer_.set(speed_ratio);
  }

  SawyerPositionController::CommandsPtr SawyerPositionController::cmdPositionMode(const intera_core_msgs::JointCommandConstPtr& msg) {
    if (msg->names.size() != msg->position.size()) {
      ROS_ERROR_STREAM_NAMED(JOINT_ARRAY_CONTROLLER_NAME, "Position commands size does not match joints size");
    }

    std::vector<double> delta_time(msg->names.size());
    std::vector<double> current_position(msg->names.size());
    double delta_time_max = 0.0;
    // Determine the time required for the Joint that will take the longest to achieve goal
    for (size_t i = 0; i < msg->names.size(); i++)
    {
      current_position[i] = controllers_[msg->names[i]]->joint_.getPosition();
      if(controllers_[msg->names[i]]->joint_urdf_->limits->velocity > 0){
        delta_time[i] = std::abs(msg->position[i] - current_position[i]) /
                     (controllers_[msg->names[i]]->joint_urdf_->limits->velocity);
      }
      else {
        delta_time[i] = 0.0;
      }
      if (delta_time[i] > delta_time_max) {
        delta_time_max = delta_time[i];
      }
    }
    CommandsPtr commands(new std::vector<Command>());
    std::shared_ptr<const std_msgs::Float64> speed_ratio;
    speed_ratio_buffer_.get(speed_ratio);
    double velocity_direction;
    // Command Joint Position and
    // Command Velocity proportional to amount of time it will take the longest
    // joint to achieve the position goal
    for (size_t i = 0; i < msg->names.size(); i++) {
      Command cmd = Command();
      cmd.name_ = msg->names[i];
      cmd.position_ = msg->position[i];
      if (delta_time_max > 0.0)
      {
        velocity_direction = (cmd.position_ <  current_position[i]) ? -1.0 : 1.0;
        if (std::abs(cmd.position_ - current_position[i]) < 0.01)
        {
          velocity_direction = 0.0;
        }
        cmd.velocity_ = speed_ratio->data * controllers_[msg->names[i]]->joint_urdf_->limits->velocity *
                        velocity_direction * delta_time[i] / delta_time_max;
        cmd.has_velocity_ = true;
      }
      commands->push_back(cmd);
    }
    return commands;
  }

  SawyerPositionController::CommandsPtr SawyerPositionController::cmdTrajectoryMode(const intera_core_msgs::JointCommandConstPtr& msg) {
      CommandsPtr commands(new std::vector<Command>());
      if (msg->names.size() != msg->position.size() || msg->names.size() != msg->velocity.size()) {
        ROS_ERROR_STREAM_NAMED(JOINT_ARRAY_CONTROLLER_NAME, "Trajectory commands size does not match joints size");
      }

      for (size_t i = 0; i < msg->names.size(); i++) {
        Command cmd = Command();
        cmd.name_ = msg->names[i];
        cmd.position_ = msg->position[i];
        cmd.velocity_ = msg->velocity[i];
        cmd.has_velocity_ = true;
        /*cmd.acceleration_ = msg->acceleration[i];
        cmd.has_acceleration_=true;*/ // TODO: Implement Feed Forward with Acceleration
        commands->push_back(cmd);
      }
      return commands;
  }

  void SawyerPositionController::jointCommandCB(const intera_core_msgs::JointCommandConstPtr& msg) {
    // lock out other thread(s) which are getting called back via ros.
    std::lock_guard<std::mutex> guard(mtx_);
    CommandsPtr commands;
    // TODO: Verify commands are valid (names of joints are correct, within URDF limits, lengths of command fields are the same)
    if(msg->mode == intera_core_msgs::JointCommand::POSITION_MODE) {
      commands = cmdPositionMode(msg);
    }
    else if(msg->mode == intera_core_msgs::JointCommand::TRAJECTORY_MODE) {
      commands = cmdTrajectoryMode(msg);
    }
    // Only write the command if this is the correct command mode, with a valid CommandPtr
    if(commands.get()) {
      command_buffer_.writeFromNonRT(*commands.get());
      new_command_ = true;
    }
  }

  void SawyerPositionController::setCommands() {
    // set the new commands for each controller
    std::vector<Command> command = *(command_buffer_.readFromRT());
    for (auto it = command.begin(); it != command.end(); it++) {
      if (it->has_velocity_) {
        controllers_[it->name_]->setCommand(it->position_, it->velocity_);
      } else {
        controllers_[it->name_]->setCommand(it->position_);
      }
    }
  }
}

PLUGINLIB_EXPORT_CLASS(sawyer_sim_controllers::SawyerPositionController, controller_interface::ControllerBase)
