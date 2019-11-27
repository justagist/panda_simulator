/***************************************************************************

*
* @package: panda_sim_controllers
* @metapackage: panda_simulator
* @author: Saif Sidhik <sxs1412@bham.ac.uk>
*

**************************************************************************/

/***************************************************************************
* Copyright (c) 2019, Saif Sidhik
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

#include <panda_sim_controllers/panda_gripper_controller.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>
#include <std_msgs/String.h>
#include <pluginlib/class_list_macros.h>
#include <yaml-cpp/yaml.h>

namespace panda_sim_controllers {

PandaGripperController::PandaGripperController()
    : new_command_(true),
      update_counter_(0),
      mimic_idx_(0),
      main_idx_(0),
      start_position_(0),
      last_position_(start_position_),
      homing_action_server(
      nh_, "/franka_gripper/homing",
      boost::bind(&PandaGripperController::homing, this, _1),false),
      stop_action_server(
      nh_, "/franka_gripper/stop",
      boost::bind(&PandaGripperController::stop, this, _1),false),
      move_action_server(
      nh_, "/franka_gripper/move",
      boost::bind(&PandaGripperController::move, this, _1), false),
      grasp_action_server(
      nh_, "/franka_gripper/grasp",
      boost::bind(&PandaGripperController::grasp, this, _1), false)
      {
}

PandaGripperController::~PandaGripperController() {
  grasp_action_server.shutdown();
  move_action_server.shutdown();
  stop_action_server.shutdown();
  homing_action_server.shutdown();
  state_pub_.shutdown();
}

bool PandaGripperController::init(panda_hardware_interface::SharedJointInterface* robot,
                                ros::NodeHandle &nh) {

  // Store nodehandle
  nh_ = nh;

  // Get joint sub-controllers
  XmlRpc::XmlRpcValue xml_struct;
  if (!nh_.getParam("joints", xml_struct)) {
    ROS_ERROR("No 'joints' parameter in controller (namespace '%s')",
              nh_.getNamespace().c_str());
    return false;
  }

  // Make sure it's a struct
  if (xml_struct.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    ROS_ERROR("The 'joints' parameter is not a struct (namespace '%s')",
              nh_.getNamespace().c_str());
    return false;
  }

  // Get number of joints
  n_joints_ = xml_struct.size();
  ROS_ASSERT_MSG(n_joints_ == 2, "Electric Gripper expects exactly 2 joints and %zu found. Exiting.", n_joints_);
  ROS_INFO_STREAM(
      "Initializing PandaGripperController with "<<n_joints_<<" joints.");

  gripper_controllers_.resize(n_joints_);
  joint_state_msg_.name.resize(n_joints_);
  joint_state_msg_.position.resize(n_joints_);
  joint_state_msg_.velocity.resize(n_joints_);
  joint_state_msg_.effort.resize(n_joints_);

  int i = 0;  
  for (const auto joint_it : xml_struct) {
    // Get joint controller
    if (joint_it.second.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_ERROR(
          "The 'joints/joint_controller' parameter is not a struct (namespace '%s')",
          nh_.getNamespace().c_str());
      return false;
    }

    // Get joint controller name
    std::string joint_controller_name = joint_it.first;
    // Get the joint-namespace nodehandle
    {
      ros::NodeHandle joint_nh(nh_, "joints/" + joint_controller_name);
      ROS_INFO_STREAM_NAMED(
          "init",
          "Loading sub-controller '" << joint_controller_name << "', Namespace: " << joint_nh.getNamespace());

      gripper_controllers_[i].reset(
          new panda_effort_controllers::JointPositionController());
      gripper_controllers_[i]->init(robot, joint_nh);

    }  // end of joint-namespaces
    // Set mimic indices
    if ( gripper_controllers_[i]->joint_urdf_->mimic ) {
      mimic_idx_ = i;
    }
    else{
      main_idx_ = i;
    }
    joint_state_msg_.name[i] = gripper_controllers_[i]->getJointName();
    // increment joint i
    ++i;
  }
  gripper_pos_command_buffer_.set(std::make_shared<double>(start_position_));
  gripper_speed_command_buffer_.set(std::make_shared<double>(start_position_));
  // std::string command_topic_name;

  
  double default_speed(0.1);
  if (nh_.getParam("default_speed", default_speed)) {
    ROS_INFO_STREAM("PandaGripperController: Found default_speed " << default_speed);
  }

  homing_action_server.start();
  stop_action_server.start();
  move_action_server.start();
  grasp_action_server.start();
  // gripper_command_action_server.start();

  //   double publish_rate(30.0);
  std::string pub_topic;
  if (nh_.getParam("pub_topic", pub_topic))
  {// They provided a custom topic to subscribe to
    // Get a node handle that is relative to the base path
    ros::NodeHandle nh_base("~");
    // Create command subscriber custom to panda
    state_pub_ = nh_base.advertise<sensor_msgs::JointState>(pub_topic, 1, true);
  }
  else
  {
    state_pub_ = nh.advertise<sensor_msgs::JointState>("/franka_gripper/joint_states", 1, true);
  }

  pub_timer_ = nh.createTimer(ros::Duration(0.1), &PandaGripperController::timerUpdate, this);
  return true;
}

void PandaGripperController::timerUpdate(const ros::TimerEvent& event) {
    // Publish at 10Hz
    
    {
    std::lock_guard<std::mutex> lock(gripper_state_mutex_);
    joint_state_msg_.position[main_idx_] = gripper_controllers_[main_idx_]->getPosition();
    joint_state_msg_.position[mimic_idx_] = gripper_controllers_[mimic_idx_]->getPosition();
    joint_state_msg_.velocity[main_idx_] = 0.0;
    joint_state_msg_.velocity[mimic_idx_] = 0.0;
    joint_state_msg_.effort[main_idx_] = 0.0;
    joint_state_msg_.effort[mimic_idx_] = 0.0;

    state_pub_.publish(joint_state_msg_);

    }
}

void PandaGripperController::update(const ros::Time& time, const ros::Duration& period)
{
  update_counter_++;
  //TODO: Change to ROS Param (20 Hz)
  if (update_counter_ % 5 == 0) {
    updateCommands();
  }
  // Apply joint commands
  for (size_t i = 0; i < n_joints_; i++) {
    // Update the individual joint controllers
    gripper_controllers_[i]->update(time, period);
  }
}

void PandaGripperController::updateCommands() {
  // Check if we have a new command to publish
  if (!new_command_)
    return;
  // Go ahead and assume we have proccessed the current message
  new_command_ = false;
  // Get latest command
  std::shared_ptr<double> cmd;
  std::shared_ptr<double> spd;
  gripper_pos_command_buffer_.get(cmd);
  gripper_speed_command_buffer_.get(spd);
  if(cmd.get()){
    auto cmd_position = *(cmd.get());
    auto cmd_speed = *(spd.get());
    // Update the individual joint controllers
    ROS_DEBUG_STREAM(gripper_controllers_[main_idx_]->joint_urdf_->name <<
                     "->setCommand(" << cmd_position << ")");
    gripper_controllers_[main_idx_]->setCommand(cmd_position,
        cmd_speed);
    gripper_controllers_[mimic_idx_]->setCommand(
        gripper_controllers_[mimic_idx_]->joint_urdf_->mimic->multiplier*
        cmd_position+gripper_controllers_[mimic_idx_]->joint_urdf_->mimic->offset,
        cmd_speed);
  }
}


void PandaGripperController::move(const franka_gripper::MoveGoalConstPtr& goal) {
  {
    std::lock_guard<std::mutex> lock(gripper_state_mutex_);
  auto cmd_position = std::min(std::max(goal->width/2.0,
                                        gripper_controllers_[main_idx_]->joint_urdf_->limits->lower),
                                        gripper_controllers_[main_idx_]->joint_urdf_->limits->upper);
  auto cmd_velocity = std::min(std::max(goal->speed,
                                        -gripper_controllers_[main_idx_]->joint_urdf_->limits->velocity),
                                        gripper_controllers_[main_idx_]->joint_urdf_->limits->velocity);
  gripper_pos_command_buffer_.set(std::make_shared<double>(cmd_position));
  gripper_speed_command_buffer_.set(std::make_shared<double>(cmd_velocity));

  new_command_ = true;
  }
  double tolerance = 0.01;
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(5.0); // Timeout of 2 seconds

  franka_gripper::MoveResult result;

  result.success = false;
  while(ros::Time::now() - start_time < timeout) {
    // do your stuff
    double current_position = std::abs(gripper_controllers_[main_idx_]->getPosition())+
                     std::abs(gripper_controllers_[mimic_idx_]->getPosition());
   if (std::abs(current_position - goal->width) <= tolerance){
    result.success = true;
    break;
    // return;
   }
  }

  if (result.success)
    move_action_server.setSucceeded(result);
  else move_action_server.setAborted(result);


  // return true;

}

void PandaGripperController::homing(const franka_gripper::HomingGoalConstPtr& /*goal*/) {
  
}

void PandaGripperController::stop(const franka_gripper::StopGoalConstPtr& /*goal*/) {
  {
  std::lock_guard<std::mutex> lock(gripper_state_mutex_);

  double current_position = std::abs(gripper_controllers_[main_idx_]->getPosition())+
                     std::abs(gripper_controllers_[mimic_idx_]->getPosition());

  gripper_pos_command_buffer_.set(std::make_shared<double>(current_position/2.0));
  gripper_speed_command_buffer_.set(std::make_shared<double>(0.0));


  new_command_ = true;
  }
  franka_gripper::StopResult result;
  result.success = true;
  stop_action_server.setSucceeded(result);
  // return true;
}

void PandaGripperController::grasp(const franka_gripper::GraspGoalConstPtr& goal) {
  {
  std::lock_guard<std::mutex> lock(gripper_state_mutex_);
  auto cmd_position = std::min(std::max(goal->width/2.0,
                                        gripper_controllers_[main_idx_]->joint_urdf_->limits->lower),
                                        gripper_controllers_[main_idx_]->joint_urdf_->limits->upper);
  auto cmd_velocity = std::min(std::max(goal->speed,
                                        -gripper_controllers_[main_idx_]->joint_urdf_->limits->velocity),
                                        gripper_controllers_[main_idx_]->joint_urdf_->limits->velocity);
  gripper_pos_command_buffer_.set(std::make_shared<double>(cmd_position));
  gripper_speed_command_buffer_.set(std::make_shared<double>(cmd_velocity));

  new_command_ = true;
  }
  double tolerance = 0.01;
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(5.0); // Timeout of 2 seconds

  franka_gripper::GraspResult result;

  result.success = false;
  while(ros::Time::now() - start_time < timeout) {
    // do your stuff
    double current_position = std::abs(gripper_controllers_[main_idx_]->getPosition())+
                     std::abs(gripper_controllers_[mimic_idx_]->getPosition());
   if (std::abs(current_position - goal->width) <= tolerance){
    result.success = true;
    break;
   }
  }

  if (result.success)
    grasp_action_server.setSucceeded(result);
  else grasp_action_server.setAborted(result);

  // return true;
}


}  // namespace

PLUGINLIB_EXPORT_CLASS(panda_sim_controllers::PandaGripperController,
                       controller_interface::ControllerBase)
