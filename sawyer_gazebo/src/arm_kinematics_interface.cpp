/***************************************************************************
* Copyright (c) 2018, Rethink Robotics Inc.
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

#include <sawyer_gazebo/arm_kinematics_interface.h>

#include <memory>

#include <intera_core_msgs/EndpointState.h>
#include <intera_core_msgs/EndpointStates.h>
#include <geometry_msgs/PoseStamped.h>

#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>

namespace sawyer_gazebo
{

bool ArmKinematicsInterface::init(ros::NodeHandle& nh, std::string side)
{
  side_ = side;
  if (!parseParams(nh))
  {
    return false;
  }

  // Limits must be retrieved before any Kinematics object is created
  joint_limits_ = retrieveJointLimits();

  // Init Solvers to default endpoint
  if (!createKinematicChain(tip_name_))
  {
    return false;
  }
    // Init gravity solvers, if not the same frame as the regular tip
  if (tip_name_ != gravity_tip_name_ && !createKinematicChain(gravity_tip_name_))
  {
    return false;
  }
  // Init Solvers to default hand camera
  // if (!createKinematicChain(camera_name_))
  // {
  //   return false;
  // }
  gravity_torques_seq_ = 0;
  endpoint_state_seq_ = 0;
  gravity_torques_pub_ = nh.advertise<intera_core_msgs::SEAJointState>(
                            "limb/"+side_+"/gravity_compensation_torques", 1);
  endpoint_state_pub_ = nh.advertise<intera_core_msgs::EndpointState>(
                            "limb/"+side_+"/endpoint_state", 1);
  tip_state_pub_ = nh.advertise<intera_core_msgs::EndpointStates>(
                            "limb/"+side_+"/tip_states", 1);
  joint_state_sub_ = nh.subscribe("joint_states", 1,
                       &ArmKinematicsInterface::jointStateCallback, this);
  joint_command_sub_ = nh.subscribe("limb/"+side_+"/joint_command", 1,
                       &ArmKinematicsInterface::jointCommandCallback, this);
  fk_service_ = nh.advertiseService(
                    "/ExternalTools/"+side_+"/PositionKinematicsNode/FKService",
                    &ArmKinematicsInterface::servicePositionFK, this);
  ik_service_ = nh.advertiseService(
                  "/ExternalTools/"+side_+"/PositionKinematicsNode/IKService",
                  &ArmKinematicsInterface::servicePositionIK, this);
  // Update at 100Hz
  update_timer_ = nh.createTimer(100, &ArmKinematicsInterface::update, this);
  joint_limits_pub_ = nh.advertise<intera_core_msgs::JointLimits>(
                            "joint_limits", 1, true);
  joint_limits_pub_.publish(joint_limits_);
  return true;
}

intera_core_msgs::JointLimits ArmKinematicsInterface::retrieveJointLimits()
{
  auto joint_limits = intera_core_msgs::JointLimits();
  // Cycle through all tree joints,
  for (const auto& kv : tree_.getSegments())
  {
    auto jnt = kv.second.segment.getJoint();
    if (jnt.getTypeName() == "None" || jnt.getTypeName() == "Unknown")
      continue;
    auto joint_name = kv.second.segment.getJoint().getName();
    auto joint_limits_ptr = robot_model_.getJoint(joint_name)->limits;
    // Save off any joint that has limits
    if (joint_limits_ptr)
    {
      joint_limits.joint_names.push_back(joint_name);
      joint_limits.position_lower.push_back(joint_limits_ptr->lower);
      joint_limits.position_upper.push_back(joint_limits_ptr->upper);
      joint_limits.velocity.push_back(joint_limits_ptr->velocity);
      joint_limits.effort.push_back(joint_limits_ptr->effort);
      // Acceleration grabbed from previously retrieved Parameter Server
      if (acceleration_map_.find(joint_name) != acceleration_map_.end())
      {
        joint_limits.accel.push_back(acceleration_map_[joint_name]);
      }
      else
      {
        ROS_INFO_NAMED("kinematics", "Unable to find Acceleration values for joint %s...",
                       joint_name.c_str());
        joint_limits.accel.push_back(8.0);
      }
    }
  }
  return joint_limits;
}


bool ArmKinematicsInterface::createKinematicChain(std::string tip_name)
{
  if(kinematic_chain_map_.find(tip_name) != kinematic_chain_map_.end())
  {
    ROS_WARN_NAMED("kinematics", "Kinematic chain from %s to %s already exists!",
                   root_name_.c_str(), tip_name.c_str());
    return false;
  }
  Kinematics kin;
  if (!tree_.getChain(root_name_, tip_name, kin.chain))
  {
    ROS_ERROR_NAMED("kinematics", "Couldn't find chain %s to %s",
                    root_name_.c_str(), tip_name.c_str());
    return false;
  }
  // Save off Joint Names
  for (size_t seg_idx = 0; seg_idx < kin.chain.getNrOfSegments(); seg_idx++)
  {
    const auto& jnt = kin.chain.getSegment(seg_idx).getJoint();
    if (jnt.getTypeName() == "None" || jnt.getTypeName() == "Unknown")
      continue;
    kin.joint_names.push_back(kin.chain.getSegment(seg_idx).getJoint().getName());
  }
  // Construct Solvers
  kin.gravity_solver = std::make_unique<KDL::ChainIdSolver_RNE>(kin.chain, KDL::Vector(0.0, 0.0, -9.8));
  kin.fk_pos_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(kin.chain);
  kin.fk_vel_solver = std::make_unique<KDL::ChainFkSolverVel_recursive>(kin.chain);
  auto num_jnts = kin.joint_names.size();
  KDL::JntArray q_min(num_jnts);
  KDL::JntArray q_max(num_jnts);
  KDL::JntArray v_max(num_jnts);
  KDL::JntArray a_max(num_jnts);
  for (size_t i = 0; i < num_jnts; i++)
  {
    auto joint_limit_idx = std::distance(joint_limits_.joint_names.begin(),
                                   std::find(joint_limits_.joint_names.begin(),
                                             joint_limits_.joint_names.end(),
                                             kin.joint_names[i]));
    if (joint_limit_idx < joint_limits_.joint_names.size())
    {
      q_min(i) = joint_limits_.position_lower[joint_limit_idx];
      q_max(i) = joint_limits_.position_upper[joint_limit_idx];
      v_max(i) = joint_limits_.velocity[joint_limit_idx];
      a_max(i) = joint_limits_.accel[joint_limit_idx];
    }
    else
    {
      ROS_WARN_NAMED("kinematics", "Unable to find Joint Limits for joint %s",
                     kin.joint_names[i].c_str());
    }
  }
  kin.ik_solver = std::make_unique<sns_ik::SNS_IK>(kin.chain, q_min, q_max,
                                                   v_max, a_max, kin.joint_names);
  kin.ik_solver->setVelocitySolveType(sns_ik::SNS_FastOptimal);
  kinematic_chain_map_.insert(std::make_pair(tip_name, std::move(kin)));
  return true;
}

bool ArmKinematicsInterface::parseParams(const ros::NodeHandle& nh)
{
  std::string urdf_xml;
  ROS_DEBUG_NAMED("kinematics", "Reading xml file from parameter server");
  if (!nh.getParam("/robot_description", urdf_xml))
  {
    ROS_FATAL_NAMED("kinematics",
        "Could not load the xml from parameter server: %s", urdf_xml.c_str());
    return false;
  }
  if (!nh.getParam("limb/"+side_+"/root_name", root_name_))
  {
    ROS_FATAL_NAMED("kinematics",
        "No root name for Kinematic Chain found on parameter server");
    return false;
  }
  if (!nh.getParam("limb/"+side_+"/tip_name", tip_name_))
  {
    ROS_FATAL_NAMED("kinematics",
        "No tip name for Kinematic Chain found on parameter server");
    return false;
  }
  if (!nh.getParam("limb/"+side_+"/gravity_tip_name", gravity_tip_name_))
  {
    ROS_FATAL_NAMED("kinematics",
        "No tip name for Kinematic Chain found on parameter server");
    return false;
  }
  // if (!nh.getParam("limb/"+side_+"/camera_name", camera_name_))
  // {
  //   ROS_FATAL_NAMED("kinematics",
  //       "No hand camera name found on parameter server");
  //   return false;
  // }
  robot_model_.initString(urdf_xml);
  if (!kdl_parser::treeFromUrdfModel(robot_model_, tree_))
  {
    ROS_FATAL_NAMED("kinematics",
        "Failed to extract kdl tree from xml robot description.");
    return false;
  }
  if (!nh.getParam("/robot_config/joint_config/joint_acceleration_limit", acceleration_map_))
  {
    ROS_FATAL_NAMED("kinematics",
      "Failed to find joint_acceleration_limit on the param server.");
    return false;
  }
  return true;
}

void ArmKinematicsInterface::update(const ros::TimerEvent& e)
{
  publishEndpointState();
  publishGravityTorques();
}

void ArmKinematicsInterface::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  auto joint_state = std::make_shared<sensor_msgs::JointState>(*msg);
  joint_state_buffer_.set(joint_state);
}

void ArmKinematicsInterface::jointCommandCallback(const intera_core_msgs::JointCommandConstPtr& msg)
{
  auto joint_command = std::make_shared<intera_core_msgs::JointCommand>(*msg);
  joint_command_buffer_.set(joint_command);
}

void ArmKinematicsInterface::publishGravityTorques()
{
  std::shared_ptr<const sensor_msgs::JointState> joint_state;
  joint_state_buffer_.get(joint_state);
  if (joint_state)
  {
    intera_core_msgs::SEAJointState gravity_torques;
    auto num_jnts = kinematic_chain_map_[tip_name_].chain.getNrOfJoints();
    KDL::JntArray jnt_pos(num_jnts), jnt_vel(num_jnts), jnt_eff(num_jnts), jnt_accelerations(num_jnts);
    KDL::JntArray jnt_gravity_model(num_jnts), jnt_gravity_only(num_jnts), jnt_zero(num_jnts);
    jointStateToKDL(*joint_state, kinematic_chain_map_[tip_name_], jnt_pos, jnt_vel, jnt_eff);

    std::shared_ptr<const intera_core_msgs::JointCommand> joint_command;
    joint_command_buffer_.get(joint_command);
    if (joint_command)
    {
      jointCommandToGravityMsg(kinematic_chain_map_[tip_name_].joint_names, *joint_command, gravity_torques);
      for (auto i = 0; i < joint_command->acceleration.size(); i++)
        jnt_accelerations(i) = joint_command->acceleration[i];
    }

    gravity_torques.name = kinematic_chain_map_[tip_name_].joint_names;
    gravity_torques.actual_position.resize(num_jnts);
    gravity_torques.actual_velocity.resize(num_jnts);
    gravity_torques.actual_effort.resize(num_jnts);
    gravity_torques.gravity_model_effort.resize(num_jnts);
    gravity_torques.gravity_only.resize(num_jnts);
    gravity_torques.interaction_torque.resize(num_jnts);
    gravity_torques.hysteresis_model_effort.resize(num_jnts);
    gravity_torques.crosstalk_model_effort.resize(num_jnts);

    computeGravity(kinematic_chain_map_[tip_name_], jnt_pos, jnt_vel, jnt_accelerations, jnt_gravity_model);
    computeGravity(kinematic_chain_map_[tip_name_], jnt_pos, jnt_zero, jnt_zero, jnt_gravity_only);
    auto clamp_limit = [](double i, double limit) { return std::max(std::min(limit, i), -limit); };
    for (size_t jnt_idx = 0; jnt_idx < num_jnts; jnt_idx++)
    {
      gravity_torques.actual_position[jnt_idx] = jnt_pos(jnt_idx);
      gravity_torques.actual_velocity[jnt_idx] = jnt_vel(jnt_idx);
      gravity_torques.actual_effort[jnt_idx] = jnt_eff(jnt_idx);
      auto torque_limit = robot_model_.getJoint(gravity_torques.name[jnt_idx])->limits->effort;
      gravity_torques.gravity_model_effort[jnt_idx] = clamp_limit(jnt_gravity_model(jnt_idx), torque_limit);
      gravity_torques.gravity_only[jnt_idx] = clamp_limit(jnt_gravity_only(jnt_idx), torque_limit);
    }
    gravity_torques.header.frame_id = root_name_;
    gravity_torques_seq_++;
    gravity_torques.header.seq = gravity_torques_seq_;
    gravity_torques.header.stamp = ros::Time::now();
    gravity_torques_pub_.publish(gravity_torques);
  }
}

void ArmKinematicsInterface::jointCommandToGravityMsg(const std::vector<std::string>& joint_names,
                                                      const intera_core_msgs::JointCommand& command_msg,
                                                      intera_core_msgs::SEAJointState& gravity_msg)
{
  auto num_jnts = joint_names.size();
  gravity_msg.commanded_position.resize(num_jnts);
  gravity_msg.commanded_velocity.resize(num_jnts);
  gravity_msg.commanded_acceleration.resize(num_jnts);
  gravity_msg.commanded_effort.resize(num_jnts);
  bool use_position = (command_msg.position.size() == command_msg.names.size() &&
                        (command_msg.mode == command_msg.POSITION_MODE ||
                         command_msg.mode == command_msg.TRAJECTORY_MODE));
  bool use_velocity = (command_msg.velocity.size() == command_msg.names.size() &&
                        (command_msg.mode == command_msg.VELOCITY_MODE ||
                         command_msg.mode == command_msg.TRAJECTORY_MODE));
  bool use_torque = (command_msg.effort.size() == command_msg.names.size() &&
                     command_msg.mode == command_msg.TORQUE_MODE);
  bool use_acceleration = (command_msg.acceleration.size() == command_msg.names.size() &&
                           command_msg.mode == command_msg.TRAJECTORY_MODE);
  for (auto i = 0; i < num_jnts; i++)
  {
    for (auto j = 0; j < command_msg.names.size(); j++)
    {
      if (command_msg.names[j] == joint_names[i])
      {
        if (use_position)
            gravity_msg.commanded_position[i] = command_msg.position[j];
        if (use_velocity)
              gravity_msg.commanded_velocity[i] = command_msg.velocity[j];
        if (use_torque)
            gravity_msg.commanded_effort[i] = command_msg.effort[j];
        if (use_acceleration)
            gravity_msg.commanded_acceleration[i] = command_msg.acceleration[j];
      }
    }
  }
}

void ArmKinematicsInterface::jointStateToKDL(const sensor_msgs::JointState& joint_state, const Kinematics& kin,
                                             KDL::JntArray& jnt_pos, KDL::JntArray& jnt_vel, KDL::JntArray& jnt_eff)
{
  auto num_jnts = kin.joint_names.size();
  auto num_msg = joint_state.name.size();
  // Check to see if there are any values before allocating space
  if (!joint_state.position.empty())
  {
    jnt_pos.resize(num_jnts);
    KDL::SetToZero(jnt_pos);
  }
  if (!joint_state.velocity.empty())
  {
    jnt_vel.resize(num_jnts);
    KDL::SetToZero(jnt_vel);
  }
  if (!joint_state.effort.empty())
  {
    jnt_eff.resize(num_jnts);
    KDL::SetToZero(jnt_eff);
  }
  for(size_t jnt_idx = 0; jnt_idx < num_jnts; jnt_idx++)
  {
    for (size_t msg_idx = 0; msg_idx < num_msg; msg_idx++)
    {
      if (joint_state.name[msg_idx] == kin.joint_names[jnt_idx])
      {
        if (msg_idx < joint_state.position.size())
          jnt_pos(jnt_idx) = joint_state.position[msg_idx];
        if (msg_idx < joint_state.velocity.size())
          jnt_vel(jnt_idx) = joint_state.velocity[msg_idx];
        if (msg_idx < joint_state.effort.size())
          jnt_eff(jnt_idx) = joint_state.effort[msg_idx];
        break;
      }
    }
  }
}

bool ArmKinematicsInterface::servicePositionIK(intera_core_msgs::SolvePositionIK::Request& req,
                                       intera_core_msgs::SolvePositionIK::Response& res)
{
  auto req_size = req.pose_stamp.size();
  res.joints.resize(req_size, sensor_msgs::JointState());
  res.result_type.resize(req_size, res.IK_FAILED);
  for (size_t i = 0; i < req_size; i++)
  {
    res.joints[i].header.stamp = ros::Time::now();
    // Try to find the kinematic chain, if not, create it
    auto kinematic_chain_it = kinematic_chain_map_.find(req.tip_names[i]);
    if (!req.tip_names.size() || kinematic_chain_it == kinematic_chain_map_.end())
    {
      if (!createKinematicChain(req.tip_names[i]))
      {
        // If chain is not found and cannot be created, leave isValid false and move on
        res.result_type[i] = res.IK_ENDPOINT_DOES_NOT_EXIST;
        continue;
      }
      else
      {
        // Update the iterator now that the chain is created
        kinematic_chain_it = kinematic_chain_map_.find(req.tip_names[i]);
      }
    }
    auto num_jnts = kinematic_chain_it->second.chain.getNrOfJoints();
    KDL::JntArray jnt_seed(num_jnts);
    KDL::JntArray jnt_result(num_jnts);
    // Nullspace
    KDL::JntArray jnt_nullspace_bias(0); // Size Zero tells SNS not to use the nullspace
    if (req.use_nullspace_goal.size() > i && req.use_nullspace_goal[i]){
      jointStatePositionToKDL(req.nullspace_goal[i],
          kinematic_chain_it->second, jnt_nullspace_bias);
      if(req.nullspace_gain.size() > i)
        kinematic_chain_it->second.ik_solver->setNullspaceGain(req.nullspace_gain[i]);
    }
    if (req.seed_mode == req.SEED_USER || req.seed_mode == req.SEED_AUTO)
    {
      if (req.seed_angles.size() > i &&
          req.seed_angles[i].name.size() && req.seed_angles[i].position.size() &&
          req.seed_angles[i].name.size() == req.seed_angles[i].position.size())
      {

        jointStatePositionToKDL(req.seed_angles[i],
            kinematic_chain_it->second, jnt_seed);
        if (computePositionIK(kinematic_chain_it->second,
                req.pose_stamp[i].pose, jnt_nullspace_bias, jnt_seed, jnt_result))
        {
          res.result_type[i] = req.SEED_USER;
        }
        /* if (jointsInCollision(kinematic_chain_map_[req.tip_names[i]], jnt_result))
        {
          res.result_type[i] = res.IK_IN_COLLISION;
        }  TODO(imcmahon) Utilize FCL for collision checking
        */
      }
    }
    if (req.seed_mode == req.SEED_CURRENT ||
         (req.seed_mode == req.SEED_AUTO &&
           (res.result_type[i] == res.IK_FAILED || res.result_type[i] == res.IK_IN_COLLISION)
         )
       )
    {
        std::shared_ptr<const sensor_msgs::JointState> joint_state;
        joint_state_buffer_.get(joint_state);
        if (joint_state.get())
        {
          jointStatePositionToKDL(*joint_state.get(),
              kinematic_chain_it->second, jnt_seed);
          if (computePositionIK(kinematic_chain_it->second,
              req.pose_stamp[i].pose, jnt_nullspace_bias, jnt_seed, jnt_result))
          {
            res.result_type[i] = req.SEED_CURRENT;
          }
        /* if (jointsInCollision(kinematic_chain_map_[req.tip_names[i]], jnt_result))
        {
          res.result_type[i] = res.IK_IN_COLLISION;
        }  TODO(imcmahon) Utilize FCL for collision checking
        */
       }
    }
    // Reset Nullspace Gain
    kinematic_chain_it->second.ik_solver->setNullspaceGain(1.0);
    // Result
    res.joints[i].name = kinematic_chain_it->second.joint_names;
    res.joints[i].position.resize(num_jnts);
    for(size_t j = 0; j < num_jnts; j++){
      res.joints[i].position[j] = jnt_result(j);
    }
  }
  return true;
}

bool ArmKinematicsInterface::servicePositionFK(intera_core_msgs::SolvePositionFK::Request& req,
                                       intera_core_msgs::SolvePositionFK::Response& res)
{
  auto req_size = req.configuration.size();
  res.inCollision.resize(req_size, false);
  res.isValid.resize(req_size, false);
  res.pose_stamp.resize(req_size, geometry_msgs::PoseStamped());
  for (size_t i = 0; i < req_size; i++)
  {
    res.pose_stamp[i].header.stamp = ros::Time::now();
    // Try to find the kinematic chain, if not, create it
    if (!req.tip_names.size() ||
        (kinematic_chain_map_.find(req.tip_names[i]) == kinematic_chain_map_.end() &&
        !createKinematicChain(req.tip_names[i])))
    {
        // If chain is not found and cannot be created, leave isValid false and move on
        continue;
    }
    KDL::JntArray jnt_pos;
    jointStatePositionToKDL(req.configuration[i], kinematic_chain_map_[req.tip_names[i]], jnt_pos);
    if (computePositionFK(kinematic_chain_map_[req.tip_names[i]], jnt_pos, res.pose_stamp[i].pose))
    {
      res.isValid[i] = true;
    }
  }
  return true;
}

bool ArmKinematicsInterface::computeGravity(const Kinematics& kin,
                                            const KDL::JntArray& jnt_pos,
                                            const KDL::JntArray& jnt_vel,
                                            const KDL::JntArray& jnt_accel,
                                            KDL::JntArray& jnt_torques)
{
  std::vector<KDL::Wrench> f_ext(kin.chain.getNrOfSegments(), KDL::Wrench::Zero());
  jnt_torques.resize(kin.chain.getNrOfJoints());
  return !(kin.gravity_solver->CartToJnt(jnt_pos, jnt_vel, jnt_accel, f_ext, jnt_torques) < 0);
}

bool ArmKinematicsInterface::computePositionFK(const Kinematics& kin,
                                               const KDL::JntArray& jnt_pos,
                                               geometry_msgs::Pose& result)
{
  KDL::Frame p_out;
  if (kin.fk_pos_solver->JntToCart(jnt_pos, p_out, kin.chain.getNrOfSegments()) < 0)
  {
    return false;
  }
  tf::poseKDLToMsg(p_out, result);
  return true;
}


bool ArmKinematicsInterface::computePositionIK(const Kinematics& kin,
                                               const geometry_msgs::Pose& cart_pose,
                                               const KDL::JntArray& jnt_nullspace_bias,
                                               const KDL::JntArray& jnt_seed, KDL::JntArray& result)
{
  KDL::Frame pose_kdl;
  tf::poseMsgToKDL(cart_pose, pose_kdl);
  return !(kin.ik_solver->CartToJnt(jnt_seed, pose_kdl, jnt_nullspace_bias, result) < 0);
}

bool ArmKinematicsInterface::computeVelocityFK(const Kinematics& kin,
                                               const KDL::JntArrayVel& jnt_vel,
                                               geometry_msgs::Twist& result)
{
  KDL::FrameVel v_out;
  if (kin.fk_vel_solver->JntToCart(jnt_vel, v_out, kin.chain.getNrOfSegments()) < 0)
  {
    return false;
  }
  tf::twistKDLToMsg(v_out.GetTwist(), result);
  return true;
}
/* TODO(imcmahon): once ChainFDSolverTau is upstreamed
bool ArmKinematicsInterface::computeEffortFK(const Kinematics& kin,
                                             const KDL::JntArray& jnt_pos,
                                             const KDL::JntArray& jnt_eff,
                                             geometry_msgs::Wrench& result)
{
  KDL::Wrench wrench;
  if (kin.fk_eff_solver->JntToCart(jnt_pos, jnt_eff, wrench) < 0)
  {
    return false;
  }
  tf::wrenchKDLToMsg(wrench, result);
  return true;
} */

void ArmKinematicsInterface::publishEndpointState()
{
  std::shared_ptr<const sensor_msgs::JointState> joint_state;
  joint_state_buffer_.get(joint_state);
  if (joint_state.get())
  {
    intera_core_msgs::EndpointStates endpoint_states;
    for(const auto& chain : kinematic_chain_map_)
    {
      intera_core_msgs::EndpointState endpoint_state;
      endpoint_state.valid = true;
      KDL::JntArray jnt_pos, jnt_vel, jnt_eff;
      jointStateToKDL(*joint_state.get(), chain.second, jnt_pos, jnt_vel, jnt_eff);
      if (!computePositionFK(chain.second, jnt_pos, endpoint_state.pose))
      {
        endpoint_state.valid &= false;
      }
      KDL::JntArrayVel jnt_array_vel(jnt_pos, jnt_vel);
      if (!computeVelocityFK(chain.second, jnt_array_vel, endpoint_state.twist))
      {
        endpoint_state.valid &= false;
      }
      /* TODO(imcmahon) once ChainFDSolverTau is upstreamed
      if(!computeEffortFK(kinematic_chain_map_[tip_name_], jnt_pos, jnt_eff, endpoint_state.wrench)){
        endpoint_state.valid &= false;
      }
      */
      endpoint_states.names.push_back(chain.first);
      endpoint_states.states.push_back(endpoint_state);
      if(chain.first == tip_name_)
      {
        endpoint_state.header.frame_id = root_name_;
        endpoint_state.header.stamp = ros::Time::now();
        endpoint_state_pub_.publish(endpoint_state);
      }
    }
    endpoint_states.header.frame_id = root_name_;
    endpoint_states.header.stamp = ros::Time::now();
    tip_state_pub_.publish(endpoint_states);
  }
}

}  // namespace sawyer_gazebo
