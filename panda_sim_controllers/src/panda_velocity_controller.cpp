/***************************************************************************
* Adapted from sawyer_velocity_controller.cpp (sawyer_simulator package)

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

#include <panda_sim_controllers/panda_velocity_controller.h>
#include <pluginlib/class_list_macros.h>

namespace panda_sim_controllers {
  bool PandaVelocityController::init(panda_hardware_interface::SharedJointInterface* hw, ros::NodeHandle &n){
    if(!panda_sim_controllers::JointArrayController<panda_effort_controllers::JointVelocityController>::init(hw, n)) {
      return false;
    } else {
      std::string topic_name;
      if (n.getParam("topic", topic_name)) {
        ros::NodeHandle nh("~");
        sub_joint_command_ = nh.subscribe(topic_name, 1, &PandaVelocityController::jointCommandCB, this);
      } else {
        sub_joint_command_ = n.subscribe("arm/joint_command", 1, &PandaVelocityController::jointCommandCB, this);
      }
      std::string topic_joint_controller_gains;
      if (n.getParam("topic_joint_controller_gains", topic_joint_controller_gains)) {
        ros::NodeHandle nh("~");
        sub_joint_ctrl_gains_ = nh.subscribe(topic_joint_controller_gains, 1, &PandaVelocityController::jointCtrlGainsCB, this);
      } else {
        sub_joint_ctrl_gains_ = n.subscribe("arm/joint_velocity_control_gains", 1, &PandaVelocityController::jointCtrlGainsCB, this);
      }
              // Start realtime state publisher
      controller_states_publisher_.reset(
      new realtime_tools::RealtimePublisher<franka_core_msgs::JointControllerStates>(n, "/arm/joint_controller_states", 1));
      if (!n.getParam("/robot_config/joint_names", controller_states_publisher_->msg_.names) ) {
      ROS_ERROR(
          "PandaVelocityController: Invalid or no joint_names parameters provided, aborting "
          "controller init!");
      return false;
      }
      
      t_ = boost::thread(&PandaVelocityController::publishControllerState, this);
    }
    // start joint_controller_states publisher
    return true;
  }

  void PandaVelocityController::jointCtrlGainsCB(const franka_core_msgs::JointControllerStatesConstPtr& msg) {
    for (size_t i = 0; i < msg->names.size(); i++)
    {
      controllers_[msg->names[i]]->setGains(msg->joint_controller_states[i].p, msg->joint_controller_states[i].i, 
                                                   msg->joint_controller_states[i].d, msg->joint_controller_states[i].i_clamp, 
                                                   0, msg->joint_controller_states[i].antiwindup);
    }
  }

  void PandaVelocityController::publishControllerState(){

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

  void PandaVelocityController::jointCommandCB(const franka_core_msgs::JointCommandConstPtr& msg) {
    if (msg->mode == franka_core_msgs::JointCommand::VELOCITY_MODE) {
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


  void PandaVelocityController::setCommands() {
    // set the new commands for each controller
    std::vector<Command> command = *(command_buffer_.readFromRT());
    for (auto it = command.begin(); it != command.end(); it++) {
      controllers_[it->name_]->setCommand(it->velocity_);
    }
  }
}

PLUGINLIB_EXPORT_CLASS(panda_sim_controllers::PandaVelocityController, controller_interface::ControllerBase)
