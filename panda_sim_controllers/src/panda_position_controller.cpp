/***************************************************************************
* Adapted from sawyer_position_controller.cpp (sawyer_simulator package)

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

#include <panda_sim_controllers/panda_position_controller.h>
#include <pluginlib/class_list_macros.h>

namespace panda_sim_controllers {
  bool PandaPositionController::init(panda_hardware_interface::SharedJointInterface* hw, ros::NodeHandle &n){
    if(!panda_sim_controllers::JointArrayController<panda_effort_controllers::JointPositionController>::init(hw, n)) {
      return false;
    } else {
      std::string topic_name;
      if (n.getParam("topic_joint_command", topic_name)) {
        ros::NodeHandle nh("~");
        sub_joint_command_ = nh.subscribe(topic_name, 1, &PandaPositionController::jointCommandCB, this);
      } else {
        sub_joint_command_ = n.subscribe("arm/joint_command", 1, &PandaPositionController::jointCommandCB, this);
      }
      std::string topic_speed_ratio;
      if (n.getParam("topic_set_speed_ratio", topic_speed_ratio)) {
        ros::NodeHandle nh("~");
        sub_speed_ratio_ = nh.subscribe(topic_speed_ratio, 1, &PandaPositionController::speedRatioCallback, this);
      } else {
        sub_speed_ratio_ = n.subscribe("arm/set_speed_ratio", 1, &PandaPositionController::speedRatioCallback, this);
      }
      std::string topic_joint_controller_gains;
      if (n.getParam("topic_joint_controller_gains", topic_joint_controller_gains)) {
        ros::NodeHandle nh("~");
        sub_joint_ctrl_gains_ = nh.subscribe(topic_joint_controller_gains, 1, &PandaPositionController::jointCtrlGainsCB, this);
      } else {
        sub_joint_ctrl_gains_ = n.subscribe("arm/joint_position_control_gains", 1, &PandaPositionController::jointCtrlGainsCB, this);
      }
      std::shared_ptr<std_msgs::Float64> speed_ratio(new std_msgs::Float64());
      speed_ratio->data = 0.3; // Default to 30% max urdf speed
      speed_ratio_buffer_.set(speed_ratio);

        // Start realtime state publisher
      controller_states_publisher_.reset(
          new realtime_tools::RealtimePublisher<franka_core_msgs::JointControllerStates>(n, "/panda_simulator/motion_controller/arm/joint_controller_states", 1));

      if (!n.getParam("/robot_config/joint_names", controller_states_publisher_->msg_.names) ) {
      ROS_ERROR(
          "PandaPositionController: Invalid or no joint_names parameters provided, aborting "
          "controller init!");
      return false;
      }
      controller_states_publisher_->msg_.controller_name = "position_joint_position_controller";

      // start joint_controller_states publisher
      t_ = boost::thread(&PandaPositionController::publishControllerState, this);


    }
    return true;
  }

  void PandaPositionController::publishControllerState(){

  ros::Rate loop_rate(50);

  // double _unused;
  while (ros::ok())
  {
    if ((controller_states_publisher_->trylock()) && (controller_states_publisher_->msg_.names.size() <= controllers_.size())) {
      std::vector<control_msgs::JointControllerState> jcs_vec;
      for (size_t i = 0; i < controller_states_publisher_->msg_.names.size(); i++)
      {
          jcs_vec.push_back(controllers_[controller_states_publisher_->msg_.names[i]]->getCurrentControllerState());
      } 
      controller_states_publisher_->msg_.joint_controller_states = jcs_vec;
      controller_states_publisher_->msg_.header.stamp = ros::Time::now();
      controller_states_publisher_->unlockAndPublish();
    }
     
    loop_rate.sleep();
  }

  }

  void PandaPositionController::jointCtrlGainsCB(const franka_core_msgs::JointControllerStatesConstPtr& msg) {
    for (size_t i = 0; i < msg->names.size(); i++)
    {
      controllers_[msg->names[i]]->setGains(msg->joint_controller_states[i].p, msg->joint_controller_states[i].i, 
                                                   msg->joint_controller_states[i].d, msg->joint_controller_states[i].i_clamp, 
                                                   0, msg->joint_controller_states[i].antiwindup);
    }
  }

  void PandaPositionController::speedRatioCallback(const std_msgs::Float64 msg) {
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
    ROS_INFO_STREAM(speed_ratio->data);
    speed_ratio_buffer_.set(speed_ratio);
  }

  PandaPositionController::CommandsPtr PandaPositionController::cmdPositionMode(const franka_core_msgs::JointCommandConstPtr& msg) {
    if (msg->names.size() != msg->position.size()) {
      ROS_ERROR_STREAM_NAMED("PandaPositionController", "Position commands size does not match joints size");
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

  PandaPositionController::CommandsPtr PandaPositionController::cmdTrajectoryMode(const franka_core_msgs::JointCommandConstPtr& msg) {
      CommandsPtr commands(new std::vector<Command>());
      // ROS_WARN_STREAM(msg->names.size() << " " << msg->position.size() << " " << msg->velocity.size());
      if (msg->names.size() != msg->position.size() || msg->names.size() != msg->velocity.size()) {
        ROS_ERROR_STREAM_NAMED("PandaPositionController", "Trajectory commands size does not match joints size");
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

  void PandaPositionController::jointCommandCB(const franka_core_msgs::JointCommandConstPtr& msg) {
    // lock out other thread(s) which are getting called back via ros.
    std::lock_guard<std::mutex> guard(mtx_);
    CommandsPtr commands;
    // TODO: Verify commands are valid (names of joints are correct, within URDF limits, lengths of command fields are the same)
    if(msg->mode == franka_core_msgs::JointCommand::POSITION_MODE) {
      commands = cmdPositionMode(msg);
    }
    else if(msg->mode == franka_core_msgs::JointCommand::IMPEDANCE_MODE) {
      commands = cmdTrajectoryMode(msg);
    }
    // Only write the command if this is the correct command mode, with a valid CommandPtr
    if(commands.get()) {
      command_buffer_.writeFromNonRT(*commands.get());
      new_command_ = true;
    }
  }

  void PandaPositionController::setCommands() {
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

PLUGINLIB_EXPORT_CLASS(panda_sim_controllers::PandaPositionController, controller_interface::ControllerBase)
