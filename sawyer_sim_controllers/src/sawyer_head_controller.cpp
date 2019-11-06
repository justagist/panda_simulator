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

#include <sawyer_sim_controllers/sawyer_head_controller.h>
#include <pluginlib/class_list_macros.h>
#include <intera_core_msgs/HeadState.h>


namespace sawyer_sim_controllers
{
const double SawyerHeadController::HEAD_PAN_DEADBAND_RADIANS = 0.01;
const double SawyerHeadController::HEAD_PAN_DEADBAND_VELOCITY = HEAD_PAN_DEADBAND_RADIANS * 2;

SawyerHeadController::SawyerHeadController() : new_command(true), update_counter(0)
{
}

SawyerHeadController::~SawyerHeadController()
{
  head_command_sub.shutdown();
}

bool SawyerHeadController::init(sawyer_hardware_interface::SharedJointInterface* robot, ros::NodeHandle& nh)
{
  // Store nodehandle
  nh_ = nh;
  // Get joint sub-controllers
  XmlRpc::XmlRpcValue xml_struct;
  if (!nh_.getParam("joints", xml_struct))
  {
    ROS_ERROR_NAMED("head", "No 'joints' parameter in controller (namespace '%s')", nh_.getNamespace().c_str());
    return false;
  }
  // Make sure it's a struct
  if (xml_struct.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR_NAMED("head", "The 'joints' parameter is not a struct (namespace '%s')", nh_.getNamespace().c_str());
    return false;
  }
  // Get number of joints
  n_joints = xml_struct.size();
  ROS_ASSERT_MSG(n_joints == 1, "SawyerHeadController only functions with one joint. Exiting.");
  ROS_INFO_STREAM_NAMED("head", "Initializing SawyerHeadController with " << n_joints << " joints.");
  for (auto joint_it : xml_struct)
  {
    // Get joint controller
    if (joint_it.second.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR_NAMED("head", "The 'joints/joint_controller' parameter is not a struct (namespace '%s')",
                      nh_.getNamespace().c_str());
      return false;
    }
    // Get joint controller name
    std::string joint_controller_name = joint_it.first;
    // Get the joint-namespace nodehandle
    {
      ros::NodeHandle joint_nh(nh_, "joints/" + joint_controller_name);
      ROS_DEBUG_STREAM_NAMED("init", "Loading sub-controller '" << joint_controller_name
                             << "', Namespace: " << joint_nh.getNamespace());
      head_controller_ptr = std::make_shared<sawyer_effort_controllers::JointPositionController>();
      head_controller_ptr->init(robot, joint_nh);
    }  // end of joint-namespaces
  }

  // Get controller topic name that it will subscribe to
  std::string cmd_topic_name;
  if (nh_.getParam("topic_command", cmd_topic_name))
  {  // They provided a custom topic to subscribe to
    // Get a node handle that is relative to the base path
    ros::NodeHandle nh_base("~");
    // Create command subscriber custom to sawyer
    head_command_sub =
        nh_base.subscribe<intera_core_msgs::HeadPanCommand>(cmd_topic_name, 1, &SawyerHeadController::commandCB, this);
  }
  else  // default "command" topic
  {
    // Create command subscriber custom to sawyer
    head_command_sub =
        nh_.subscribe<intera_core_msgs::HeadPanCommand>("command", 1, &SawyerHeadController::commandCB, this);
  }
  std::string state_topic_name;
  if (nh_.getParam("topic_state", state_topic_name))
  {// They provided a custom topic to subscribe to
    // Get a node handle that is relative to the base path
    ros::NodeHandle nh_base("~");
    // Create command subscriber custom to sawyer
    head_state_pub_ = nh_base.advertise<intera_core_msgs::HeadState>(state_topic_name, 1);
  }
  else
  {
    head_state_pub_ = nh.advertise<intera_core_msgs::HeadState>("state", 1);
  }

  // Update at 100Hz
  head_state_pub_timer_ = nh.createTimer(100, &SawyerHeadController::publishHeadState, this);
  return true;
}

void SawyerHeadController::publishHeadState(const ros::TimerEvent& event)
{
  auto head_state = intera_core_msgs::HeadState();
  head_state.pan = head_controller_ptr->getPosition();
  head_state.isTurning = std::abs(head_controller_ptr->joint_.getVelocity()) > HEAD_PAN_DEADBAND_VELOCITY;
  head_state.isBlocked = false;
  head_state.panMode = head_state.ACTIVE_MODE;
  head_state_pub_.publish(head_state);
}

void SawyerHeadController::starting(const ros::Time& time)
{
  auto initial_command = std::make_shared<HeadCommand>();
  // Fill in the initial command
  for (std::size_t i = 0; i < n_joints; i++)
  {
    initial_command->position = head_controller_ptr->getPosition();
    initial_command->velocity = 0.0;
  }
  head_command_buffer.initRT(initial_command);
  new_command = true;
}

void SawyerHeadController::stopping(const ros::Time& time)
{
}

void SawyerHeadController::update(const ros::Time& time, const ros::Duration& period)
{
  update_counter++;
  if (update_counter % 100 == 0)
    updateCommands();

  // Update the individual joint controllers
  head_controller_ptr->update(time, period);
}

void SawyerHeadController::updateCommands()
{
  // Check if we have a new command send to the hardware
  if (!new_command)
    return;
  // Go ahead and assume we have proccessed the current message
  new_command = false;
  // Get latest command
  const auto& head_command = *(head_command_buffer.readFromRT());
  head_controller_ptr->setCommand(head_command->position, head_command->velocity);
}

void SawyerHeadController::commandCB(const intera_core_msgs::HeadPanCommandConstPtr& msg)
{
  // Command Head Pan Position and
  // Command Velocity proportional to speed ratio x direction x URDF max speed
  // to achieve the position goal
  auto cmd_ptr = std::make_shared<HeadCommand>();
  double speed_ratio = std::max(std::min(msg->speed_ratio, msg->MAX_SPEED_RATIO), msg->MIN_SPEED_RATIO);
  cmd_ptr->position = std::max(std::min(static_cast<double>(msg->target), static_cast<double>(head_controller_ptr->joint_urdf_->limits->upper)),
                               static_cast<double>(head_controller_ptr->joint_urdf_->limits->lower));
  auto current_position = head_controller_ptr->joint_.getPosition();
  double velocity_direction = (cmd_ptr->position < current_position) ? -1.0 : 1.0;
  if (std::abs(cmd_ptr->position - current_position) < HEAD_PAN_DEADBAND_RADIANS)
  {
    velocity_direction = 0.0;
  }
  cmd_ptr->velocity = speed_ratio * head_controller_ptr->joint_urdf_->limits->velocity * velocity_direction;
  head_command_buffer.writeFromNonRT(cmd_ptr);
  new_command = true;
}

}  // namespace

PLUGINLIB_EXPORT_CLASS(sawyer_sim_controllers::SawyerHeadController, controller_interface::ControllerBase)
