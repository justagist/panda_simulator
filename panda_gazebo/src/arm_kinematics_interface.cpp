/***************************************************************************
* Adapted from arm_kinematics_interface.cpp (sawyer_simulator package)

*
* @package: panda_gazebo
* @metapackage: panda_simulator
* @author: Saif Sidhik <sxs1412@bham.ac.uk>
*

**************************************************************************/

/***************************************************************************
* Copyright (c) 2019-2020, Saif Sidhik
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

#include <panda_gazebo/arm_kinematics_interface.h>

#include <memory>

#include <franka_core_msgs/EndPointState.h>
#include <geometry_msgs/PoseStamped.h>

#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>

namespace panda_gazebo
{

bool ArmKinematicsInterface::init(ros::NodeHandle& nh)
{
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
  robot_state_publisher_ = nh.advertise<franka_core_msgs::RobotState>(
                            "/panda_simulator/custom_franka_state_controller/robot_state", 1);
  endpoint_state_pub_ = nh.advertise<franka_core_msgs::EndPointState>(
                            "/panda_simulator/custom_franka_state_controller/tip_state", 1);
  joint_state_sub_ = nh.subscribe("joint_states", 1,
                       &ArmKinematicsInterface::jointStateCallback, this);
  joint_command_sub_ = nh.subscribe("/panda_simulator/motion_controller/arm/joint_commands", 1,
                       &ArmKinematicsInterface::jointCommandCallback, this);
  // Update at 100Hz
  update_timer_ = nh.createTimer(100, &ArmKinematicsInterface::update, this);
  joint_limits_pub_ = nh.advertise<franka_core_msgs::JointLimits>(
                            "/panda_simulator/joint_limits", 1, true);
  joint_limits_pub_.publish(joint_limits_);
  return true;
}

franka_core_msgs::JointLimits ArmKinematicsInterface::retrieveJointLimits()
{
  auto joint_limits = franka_core_msgs::JointLimits();
  // Cycle through all tree joints,
  for (const auto& kv : tree_.getSegments())
  {
    auto jnt = kv.second.segment.getJoint();
    if (jnt.getTypeName() == "None" || jnt.getTypeName() == "Unknown" || jnt.getTypeName() == "Fixed")
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
    if (jnt.getTypeName() == "None" || jnt.getTypeName() == "Unknown" || jnt.getTypeName() == "Fixed")
      continue;
    kin.joint_names.push_back(kin.chain.getSegment(seg_idx).getJoint().getName());
  }
  
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
  kdl_ = new KDLMethods;
  kdl_->initialise(kin.chain);
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
  if (!nh.getParam("/arm/root_name", root_name_))
  {
    ROS_FATAL_NAMED("kinematics",
        "No root name for Kinematic Chain found on parameter server");
    return false;
  }
  if (!nh.getParam("/arm/tip_name", tip_name_))
  {
    ROS_FATAL_NAMED("kinematics",
        "No tip name for Kinematic Chain found on parameter server");
    return false;
  }
  if (!nh.getParam("/arm/gravity_tip_name", gravity_tip_name_))
  {
    ROS_FATAL_NAMED("kinematics",
        "No tip name for Kinematic Chain found on parameter server");
    return false;
  }
  // if (!nh.getParam("arm/camera_name", camera_name_))
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
  publishRobotState();
}

void ArmKinematicsInterface::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  auto joint_state = std::make_shared<sensor_msgs::JointState>(*msg);
  joint_state_buffer_.set(joint_state);
}

void ArmKinematicsInterface::jointCommandCallback(const franka_core_msgs::JointCommandConstPtr& msg)
{
  auto joint_command = std::make_shared<franka_core_msgs::JointCommand>(*msg);
  joint_command_buffer_.set(joint_command);
}

void ArmKinematicsInterface::publishRobotState()
{
  std::shared_ptr<const sensor_msgs::JointState> joint_state;
  joint_state_buffer_.get(joint_state);
  if (joint_state)
  {
    franka_core_msgs::RobotState robot_state_msg;
    auto num_jnts = kinematic_chain_map_[tip_name_].chain.getNrOfJoints();
    // ROS_WARN_NAMED("this","num_j %d", num_jnts);
    KDL::JntArray jnt_pos(num_jnts), jnt_vel(num_jnts), jnt_eff(num_jnts), jnt_accelerations(num_jnts);
    KDL::JntArray jnt_gravity_model(num_jnts), jnt_gravity_only(num_jnts), jnt_zero(num_jnts);
    jointStateToKDL(*joint_state, kinematic_chain_map_[tip_name_], jnt_pos, jnt_vel, jnt_eff);

    // std::shared_ptr<const franka_core_msgs::JointCommand> joint_command;
    // joint_command_buffer_.get(joint_command);
    // if (joint_command)
    // { 
    //   std::array<double, 7> acc;
    //   addGravityToMsg(kinematic_chain_map_[tip_name_].joint_names, *joint_command, robot_state_msg, acc);
    //   for (auto i = 0; i < joint_command->acceleration.size(); i++)
    //     jnt_accelerations(i) = joint_command->acceleration[i];
    // }
    // computeGravity(kinematic_chain_map_[tip_name_], jnt_pos, jnt_vel, jnt_accelerations, jnt_gravity_model);



    // computeGravity(kinematic_chain_map_[tip_name_], jnt_pos, jnt_zero, jnt_zero, jnt_gravity_only);
    // auto clamp_limit = [](double i, double limit) { return std::max(std::min(limit, i), -limit); };
    // for (size_t jnt_idx = 0; jnt_idx < num_jnts; jnt_idx++)
    // {
    //   auto torque_limit = robot_model_.getJoint(kinematic_chain_map_[tip_name_].joint_names[jnt_idx])->limits->effort;
    //   // std::cout << jnt_idx << " " << jnt_gravity_model(jnt_idx) << " " << jnt_gravity_only(jnt_idx) << " "<< clamp_limit(jnt_gravity_only(jnt_idx), torque_limit) <<  std::endl;
    //   robot_state_msg.gravity[jnt_idx] = clamp_limit(jnt_gravity_only(jnt_idx), torque_limit);
    //   robot_state_msg.coriolis[jnt_idx] = clamp_limit(jnt_gravity_model(jnt_idx), torque_limit) - robot_state_msg.gravity[jnt_idx];
    // }

    addJacAndVelToMsg(kinematic_chain_map_[tip_name_], jnt_pos, jnt_vel, robot_state_msg);

    addDynamicsToMsg(kinematic_chain_map_[tip_name_], jnt_pos, jnt_vel, robot_state_msg);


    robot_state_msg.header.frame_id = root_name_;
    gravity_torques_seq_++;
    robot_state_msg.header.seq = gravity_torques_seq_;
    robot_state_msg.header.stamp = ros::Time::now();
    robot_state_publisher_.publish(robot_state_msg);
  }
}
void ArmKinematicsInterface::addDynamicsToMsg(const Kinematics& kin, const KDL::JntArray& jnt_pos,
                                               const KDL::JntArray& jnt_vel, franka_core_msgs::RobotState& robot_state)
{
    auto num_jnts = kin.chain.getNrOfJoints();

    KDL::JntArray C(num_jnts); //coriolis matrix
    KDL::JntArray G(num_jnts); //gravity matrix
    KDL::JntSpaceInertiaMatrix H(num_jnts); //inertiamatrix H=square matrix of size= number of joints
    kdl_->JntToMass(jnt_pos,H);
    kdl_->JntToCoriolis(jnt_pos,jnt_vel,C);
    kdl_->JntToGravity(jnt_pos,G);

    for (size_t jnt_idx = 0; jnt_idx < num_jnts; jnt_idx++)
    {
      robot_state.gravity[jnt_idx] = G(jnt_idx,0);
      robot_state.coriolis[jnt_idx] = C(jnt_idx,0);
    }

    Eigen::Map<Eigen::RowVectorXd> H_vec(H.data.data(), H.data.size());
    for (size_t i = 0; i < robot_state.mass_matrix.size(); i++)
    {
      robot_state.mass_matrix[i] = H_vec(i);
    }
} 

void ArmKinematicsInterface::addJacAndVelToMsg(const Kinematics& kin, const KDL::JntArray& jnt_pos,
                                               const KDL::JntArray& jnt_vel, franka_core_msgs::RobotState& robot_state)
{
    KDL::Jacobian J;
    J.resize(kin.chain.getNrOfJoints());
    kdl_->JacobianJntToJac(jnt_pos, J);

    Eigen::Matrix<double, 6, 1> ee_vel = J.data * jnt_vel.data;

    Eigen::Map<Eigen::RowVectorXd> J_vec(J.data.data(), J.data.size());

    for (size_t i = 0; i < robot_state.cartesian_collision.size(); i++) {
      robot_state.O_dP_EE[i] = ee_vel(i,0);
    }
    for (size_t i = 0; i < robot_state.O_Jac_EE.size(); i++) {
      robot_state.O_Jac_EE[i] = J_vec(i);
    }

}

void ArmKinematicsInterface::addGravityToMsg(const std::vector<std::string>& joint_names,
                                                      const franka_core_msgs::JointCommand& command_msg,
                                                      franka_core_msgs::RobotState& robot_state, std::array<double, 7>& acc)
{
  auto num_jnts = joint_names.size();

  bool use_position = (command_msg.position.size() == command_msg.names.size() &&
                        (command_msg.mode == command_msg.POSITION_MODE ||
                         command_msg.mode == command_msg.IMPEDANCE_MODE));
  bool use_velocity = (command_msg.velocity.size() == command_msg.names.size() &&
                        (command_msg.mode == command_msg.VELOCITY_MODE ||
                         command_msg.mode == command_msg.IMPEDANCE_MODE));
  bool use_torque = (command_msg.effort.size() == command_msg.names.size() &&
                     command_msg.mode == command_msg.TORQUE_MODE);
  // bool use_acceleration = (command_msg.acceleration.size() == command_msg.names.size() &&
  //                          command_msg.mode == command_msg.IMPEDANCE_MODE);
  for (auto i = 0; i < num_jnts; i++)
  {
    for (auto j = 0; j < command_msg.names.size(); j++)
    {
      if (command_msg.names[j] == joint_names[i])
      {
        if (use_position)
            robot_state.q_d[i] = command_msg.position[j];
        if (use_velocity)
              robot_state.dq_d[i] = command_msg.velocity[j];
        if (use_torque)
            robot_state.tau_J_d[i] = command_msg.effort[j];
        // if (use_acceleration)
        //     acc[i] = command_msg.acceleration[j];
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


bool ArmKinematicsInterface::computeGravity(const Kinematics& kin,
                                            const KDL::JntArray& jnt_pos,
                                            const KDL::JntArray& jnt_vel,
                                            const KDL::JntArray& jnt_accel,
                                            KDL::JntArray& jnt_torques)
{
  std::vector<KDL::Wrench> f_ext(kin.chain.getNrOfSegments(), KDL::Wrench::Zero());
  jnt_torques.resize(kin.chain.getNrOfJoints());
  // return !(kdl_->GravityCartToJnt(jnt_pos, jnt_vel, jnt_accel, f_ext, jnt_torques) < 0);
  return false;
}

bool ArmKinematicsInterface::computePositionFK(const Kinematics& kin,
                                               const KDL::JntArray& jnt_pos,
                                               geometry_msgs::Pose& result)
{
  KDL::Frame p_out;
  if (kdl_->PosFKJntToCart(jnt_pos, p_out, kin.chain.getNrOfSegments()) < 0)
  {
    return false;
  }
  tf::poseKDLToMsg(p_out, result);
  return true;
}


void ArmKinematicsInterface::publishEndpointState()
{
  std::shared_ptr<const sensor_msgs::JointState> joint_state;
  joint_state_buffer_.get(joint_state);
  if (joint_state.get())
  {
    franka_core_msgs::EndPointState endpoint_state;
    endpoint_state.O_F_ext_hat_K.header.frame_id = "UNDEFINED";
    endpoint_state.O_F_ext_hat_K.wrench.force.x = 0.0;
    endpoint_state.O_F_ext_hat_K.wrench.force.y = 0.0;
    endpoint_state.O_F_ext_hat_K.wrench.force.z = 0.0;
    endpoint_state.O_F_ext_hat_K.wrench.torque.x = 0.0;
    endpoint_state.O_F_ext_hat_K.wrench.torque.y = 0.0;
    endpoint_state.O_F_ext_hat_K.wrench.torque.z = 0.0;

    endpoint_state.K_F_ext_hat_K.header.frame_id = "UNDEFINED";
    endpoint_state.K_F_ext_hat_K.wrench.force.x = 0.0;
    endpoint_state.K_F_ext_hat_K.wrench.force.y = 0.0;
    endpoint_state.K_F_ext_hat_K.wrench.force.z = 0.0;
    endpoint_state.K_F_ext_hat_K.wrench.torque.x = 0.0;
    endpoint_state.K_F_ext_hat_K.wrench.torque.y = 0.0;
    endpoint_state.K_F_ext_hat_K.wrench.torque.z = 0.0;

    for(const auto& chain : kinematic_chain_map_)
    {
      

       // TODO(imcmahon) once ChainFDSolverTau is upstreamed
      // computeEffortFK(kinematic_chain_map_[tip_name_], jnt_pos, jnt_eff, endpoint_state.O_F_ext_hat_K.wrench);
      
      if(chain.first == tip_name_)
      {
        geometry_msgs::Pose pose;
        KDL::JntArray jnt_pos, jnt_vel, jnt_eff;
        jointStateToKDL(*joint_state.get(), chain.second, jnt_pos, jnt_vel, jnt_eff);

        computePositionFK(chain.second, jnt_pos, pose);

        Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        // Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z)));
        Eigen::Matrix3d m = q.toRotationMatrix();


        Eigen::Vector3d t(pose.position.x, pose.position.y, pose.position.z);


        Eigen::Matrix4d Trans; // Your Transformation Matrix
        Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
        Trans.block<3,3>(0,0) = m;
        Trans.block<3,1>(0,3) = t;

        Eigen::Map<Eigen::RowVectorXd> flattened_mat(Trans.data(), Trans.size());
        // endpoint_state.O_T_EE = flattened_mat.data()
        std::vector<double> vec(flattened_mat.data(), flattened_mat.data() + flattened_mat.size());


        std::copy_n(vec.begin(), 16, endpoint_state.O_T_EE.begin());
        // t.linear() = q.toRotationMatrix();

        KDL::JntArrayVel jnt_array_vel(jnt_pos, jnt_vel);

        endpoint_state.header.frame_id = root_name_;
        endpoint_state.header.stamp = ros::Time::now();
        endpoint_state_pub_.publish(endpoint_state);
      }
    }
  }
}

}  // namespace panda_gazebo
