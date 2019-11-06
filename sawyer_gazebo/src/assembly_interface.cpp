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

// Overload the default plugin
#include <sawyer_gazebo/sawyer_gazebo_ros_control_plugin.h>

namespace sawyer_gazebo {

void AssemblyInterface::init(ros::NodeHandle& nh) {
    //Default values for the assembly state
    sim_estop_.data = false;
    std::shared_ptr<intera_core_msgs::AssemblyState> assembly_state(new intera_core_msgs::AssemblyState());
    assembly_state->ready = true;  // true if enabled
    assembly_state->enabled = true;  // true if enabled
    assembly_state->stopped = false; // true if stopped -- e-stop asserted
    assembly_state->error = false;  // true if a component of the assembly has an error
    assembly_state->lowVoltage = false;  // true if the robot entered lowVoltage mode
    assembly_state->estop_button = intera_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED;  // button status
    assembly_state->estop_source = intera_core_msgs::AssemblyState::ESTOP_SOURCE_NONE;  // If stopped is
    is_enabled_ = assembly_state->enabled;
    is_stopped_ = assembly_state->stopped;
    assembly_state_buffer_.set(assembly_state);
    assembly_state_pub_ = nh.advertise<intera_core_msgs::AssemblyState>("state", 1);
    assembly_sim_estop_pub_ = nh.advertise<std_msgs::Bool>("sim_estop", 1);
    assembly_state_timer_ = nh.createTimer(100, &AssemblyInterface::update, this);// 100Hz
    assembly_enable_sub_ = nh.subscribe("set_super_enable", 100, &AssemblyInterface::callbackEnable, this);
    assembly_stop_sub_ =  nh.subscribe("set_super_stop", 100, &AssemblyInterface::callbackStop, this);
    assembly_reset_sub_ = nh.subscribe("set_super_reset", 100, &AssemblyInterface::callbackReset, this);
}

void AssemblyInterface::update(const ros::TimerEvent& e){
    std::shared_ptr<const intera_core_msgs::AssemblyState> assembly_state;
    assembly_state_buffer_.get(assembly_state);
    is_enabled_ = assembly_state->enabled;
    is_stopped_ = assembly_state->stopped;
    bool status = (!is_enabled_ || is_stopped_);
    if(sim_estop_.data != status){
      sim_estop_.data = status;
      assembly_sim_estop_pub_.publish(sim_estop_);
    }
    assembly_state_pub_.publish(*assembly_state);
}

void AssemblyInterface::callbackEnable(const std_msgs::Bool &msg) {
  std::shared_ptr<intera_core_msgs::AssemblyState> assembly_state(new intera_core_msgs::AssemblyState());
  if (msg.data) {
    assembly_state->enabled = true;
  }
  else {
    assembly_state->enabled = false;
  }
  assembly_state->ready = true;
  assembly_state->stopped = false;
  assembly_state->error = false;
  assembly_state->lowVoltage = false;
  assembly_state->estop_button = intera_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED;
  assembly_state->estop_source = intera_core_msgs::AssemblyState::ESTOP_SOURCE_NONE;
  assembly_state_buffer_.set(assembly_state);
}

 /**
  * Method to stop the robot and capture the source of the stop
  */
void AssemblyInterface::callbackStop(const std_msgs::Empty &msg) {
  std::shared_ptr<intera_core_msgs::AssemblyState> assembly_state(new intera_core_msgs::AssemblyState());
  assembly_state->enabled = false;
  assembly_state->ready = false;
  assembly_state->stopped = true;
  assembly_state->error = false;
  assembly_state->lowVoltage = false;
  assembly_state->estop_button = intera_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED;
  assembly_state->estop_source = intera_core_msgs::AssemblyState::ESTOP_SOURCE_UNKNOWN;
  assembly_state_buffer_.set(assembly_state);
}

 /**
  * Method resets all the values to False and 0s
  */
void AssemblyInterface::callbackReset(const std_msgs::Empty &msg) {
  std::shared_ptr<intera_core_msgs::AssemblyState> assembly_state(new intera_core_msgs::AssemblyState());
  assembly_state->enabled = false;
  assembly_state->ready = true;
  assembly_state->stopped = false;
  assembly_state->error = false;
  assembly_state->estop_button = intera_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED;
  assembly_state->estop_source = intera_core_msgs::AssemblyState::ESTOP_SOURCE_NONE;
  assembly_state_buffer_.set(assembly_state);
}

}
