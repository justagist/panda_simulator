/***************************************************************************
* Adapted from sawyer_joint_velocity_controller.cpp (sawyer_simulator package)

*
* @package: panda_sim_controllers
* @metapackage: panda_simulator
* @author: Saif Sidhik <sxs1412@bham.ac.uk>
*

**************************************************************************/

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <panda_sim_controllers/panda_joint_velocity_controller.h>
#include <pluginlib/class_list_macros.h>


namespace panda_effort_controllers {

JointVelocityController::JointVelocityController()
: command_(0), loop_count_(0)
{}

JointVelocityController::~JointVelocityController()
{
  sub_command_.shutdown();
}

bool JointVelocityController::init(panda_hardware_interface::SharedJointInterface *robot,
  const std::string &joint_name, const control_toolbox::Pid &pid)
{
  pid_controller_ = pid;

  // Get joint handle from hardware interface
  joint_ = robot->getHandle(joint_name);

  return true;
}

bool JointVelocityController::init(panda_hardware_interface::SharedJointInterface *robot, ros::NodeHandle &n)
{
  // Get joint name from parameter server
  std::string joint_name;
  if (!n.getParam("joint", joint_name)) {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }

  // Get joint handle from hardware interface
  joint_ = robot->getHandle(joint_name);

  // Load PID Controller using gains set on parameter server
  if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
    return false;

  // Start realtime state publisher
  // controller_state_publisher_.reset(
  //   new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>
  //   (n, "state", 1));

  // Start command subscriber
  // sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &JointVelocityController::setCommandCB, this);

  return true;
}

void JointVelocityController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup)
{
  pid_controller_.setGains(p,i,d,i_max,i_min,antiwindup);
}

void JointVelocityController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
{
  pid_controller_.getGains(p,i,d,i_max,i_min,antiwindup);
}

void JointVelocityController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  bool dummy;
  pid_controller_.getGains(p,i,d,i_max,i_min,dummy);
}

void JointVelocityController::printDebug()
{
  pid_controller_.printValues();
}

control_msgs::JointControllerState JointVelocityController::getCurrentControllerState()
{
  return curr_state_;
}

std::string JointVelocityController::getJointName()
{
  return joint_.getName();
}

// Set the joint velocity command
void JointVelocityController::setCommand(double cmd)
{
  command_ = cmd;
}

// Return the current velocity command
void JointVelocityController::getCommand(double& cmd)
{
  cmd = command_;
}

void JointVelocityController::starting(const ros::Time& time)
{
  command_ = 0.0;
  pid_controller_.reset();
}

void JointVelocityController::update(const ros::Time& time, const ros::Duration& period)
{
  double error = command_ - joint_.getVelocity();

  // Set the PID error and compute the PID command with nonuniform time
  // step size. The derivative error is computed from the change in the error
  // and the timestep dt.
  double commanded_effort = pid_controller_.computeCommand(error, period);

  joint_.setCommand(commanded_effort);

  {
    double dummy;
    bool antiwindup;
    curr_state_.header.stamp = time;
    curr_state_.set_point = command_;
    curr_state_.process_value = joint_.getVelocity();
    curr_state_.error = error;
    curr_state_.time_step = period.toSec();
    curr_state_.command = commanded_effort;

    getGains(curr_state_.p, curr_state_.i, curr_state_.d, curr_state_.i_clamp, dummy, antiwindup);
    curr_state_.antiwindup = static_cast<char>(antiwindup);
  }
}

void JointVelocityController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
{
  command_ = msg->data;
}

} // namespace

PLUGINLIB_EXPORT_CLASS( panda_effort_controllers::JointVelocityController, controller_interface::ControllerBase)
