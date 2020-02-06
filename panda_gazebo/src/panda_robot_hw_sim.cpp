/***************************************************************************
* Adapted from sawyer_robot_hw_sim.cpp (sawyer_simulator package)

*
* @package: panda_gazebo
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

#include <panda_gazebo/panda_robot_hw_sim.h>

namespace panda_gazebo
{

const double PandaRobotHWSim::BRAKE_VALUE = 10000.0;

bool PandaRobotHWSim::initSim(
  const std::string& robot_namespace,
  ros::NodeHandle model_nh,
  gazebo::physics::ModelPtr parent_model,
  const urdf::Model *const urdf_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions)
{

  bool ret = gazebo_ros_control::DefaultRobotHWSim::initSim(robot_namespace,
                           model_nh, parent_model, urdf_model, transmissions);

  // add custom (sum) interfaces
  // std::cout << std::endl << "INIT " << std::endl;
  ret &= PandaRobotHWSim::initCustomInterfaces();
  // std::cout << std::endl << "INIT CUSTOM" << std::endl;

  PandaRobotHWSim::initBrakes();
  // std::cout << std::endl << "INIT BRAKES" << std::endl;
  return ret;
}



bool PandaRobotHWSim::initCustomInterfaces()
{

  // Hard-code apply SumEffortInterfaces to all joints in Panda

  // Initialize values
  for(unsigned int j=0; j < n_dof_; j++)
  {
    //   ROS_WARN_STREAM("Hey!! ");
    // ROS_WARN_STREAM("CONTROL METHOD: " << std::to_string(joint_control_methods_[j]));
    // ROS_WARN_STREAM("DESIRED METHOD: " << std::to_string(EFFORT));
    if (joint_control_methods_[j] == EFFORT)
    {
      ROS_DEBUG_STREAM_NAMED("panda_robot_hw_sim","Loading joint '" << joint_names_[j]
        << "' of type '" << "panda_hardware_interface/SharedJointInterface" << "'");

      panda_hardware_interface::SumJointHandlePtr sumHandle(new panda_hardware_interface::SumJointHandle(
        js_interface_.getHandle(joint_names_[j]),
        &joint_effort_command_[j]));

      sum_ej_handles_refs_.push_back(sumHandle);
      sum_ej_interface_.registerContainer(sumHandle);
    }
    else
    {
      ROS_FATAL_STREAM_NAMED("panda_robot_hw_sim","No matching hardware interface found for '"
        << "panda_hardware_interface/SharedJointInterface" << "' while loading interfaces for " << joint_names_[j] );
      return false;
    }

  }

  // Register interfaces
  gazebo_ros_control::DefaultRobotHWSim::registerInterface(&sum_ej_interface_);

  return true;
}

void PandaRobotHWSim::initBrakes()
{
  
  joint_enable_.resize(gazebo_ros_control::DefaultRobotHWSim::n_dof_);
  joint_disable_.resize(gazebo_ros_control::DefaultRobotHWSim::n_dof_);
  for(std::size_t i = 0; i != gazebo_ros_control::DefaultRobotHWSim::sim_joints_.size(); ++i)
  {
    joint_enable_[i] = gazebo_ros_control::DefaultRobotHWSim::sim_joints_[i]->GetDamping(0);
    if (joint_names_[i] == "panda_joint1" ||
        joint_names_[i] == "panda_joint2" ||
        joint_names_[i] == "panda_joint3"){
      // Add brakes to j1 and j2
      joint_disable_[i] = BRAKE_VALUE;
    }
    else{
      joint_disable_[i] = joint_enable_[i];
    }
  }
}


void PandaRobotHWSim::writeSim(ros::Time time, ros::Duration period)
{
  // NB: Majority of this function is copy paste of inherited writeSim

  // If the E-stop is active, joints controlled by position commands will maintain their positions.
  // ROS_WARN_STREAM("writeSim j_p_ " << std::to_string(joint_position_[0]));
  if (e_stop_active_)
  {
    if (!last_e_stop_active_)
    {
      last_joint_position_command_ = joint_position_;
      last_e_stop_active_ = true;
    }
    joint_position_command_ = last_joint_position_command_;
  }
  else
  {
    last_e_stop_active_ = false;
  }

  ej_sat_interface_.enforceLimits(period);
  ej_limits_interface_.enforceLimits(period);
  pj_sat_interface_.enforceLimits(period);
  pj_limits_interface_.enforceLimits(period);
  vj_sat_interface_.enforceLimits(period);
  vj_limits_interface_.enforceLimits(period);

  // for each handle in sum vector store, call 'updateCommandSum()'
  for (std::vector<panda_hardware_interface::SumJointHandlePtr>::iterator it = sum_ej_handles_refs_.begin(); it != sum_ej_handles_refs_.end(); ++it)
  {
    (*it)->updateCommandSum();
  }
  ROS_DEBUG_STREAM_THROTTLE_NAMED(20, "panda_robot_hw_sim", "SumJoint '" << sum_ej_handles_refs_[0]->getName() << "' has ("
      << sum_ej_handles_refs_[0]->howManySubs() << ") subs");


  for(unsigned int j=0; j < n_dof_; j++)
  {
    switch (joint_control_methods_[j])
    {
      case EFFORT:
        {
          const double effort = e_stop_active_ ? 0 : joint_effort_command_[j];
          sim_joints_[j]->SetForce(0, effort);
        }
        break;
      default:
        {
          ROS_WARN_STREAM_NAMED("panda_robot_hw_sim", "Non-EFFORT controlled joint on panda: '"
              << joint_names_[j] << "'!");
          break;
        }
    }
  }
}


void PandaRobotHWSim::brakesActive(const bool active)
{
  for(std::size_t i = 0; i != gazebo_ros_control::DefaultRobotHWSim::sim_joints_.size(); ++i)
  {
    gazebo_ros_control::DefaultRobotHWSim::sim_joints_[i]->SetDamping(0, (active ? joint_disable_[i] : joint_enable_[i]));
  }
}

void PandaRobotHWSim::eStopActive(const bool active)
{
  PandaRobotHWSim::brakesActive(active);
  gazebo_ros_control::DefaultRobotHWSim::eStopActive(active);
}

}

PLUGINLIB_EXPORT_CLASS(panda_gazebo::PandaRobotHWSim, gazebo_ros_control::RobotHWSim)
