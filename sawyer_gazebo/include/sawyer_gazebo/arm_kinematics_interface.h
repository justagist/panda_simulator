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
#ifndef SAWYER_GAZEBO_ARM_KINEMATICS_INTERFACE_H
#define SAWYER_GAZEBO_ARM_KINEMATICS_INTERFACE_H

#include <map>
#include <string>

#include <ros/ros.h>
#include <realtime_tools/realtime_box.h>

#include <sensor_msgs/JointState.h>
#include <intera_core_msgs/JointCommand.h>
#include <intera_core_msgs/JointLimits.h>
#include <intera_core_msgs/SolvePositionFK.h>
#include <intera_core_msgs/SolvePositionIK.h>
#include <intera_core_msgs/SEAJointState.h>

#include <geometry_msgs/Twist.h>

#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <sns_ik/sns_ik.hpp>

#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <urdf/model.h>

namespace sawyer_gazebo
{
class ArmKinematicsInterface
{
public:
/** Method to initialize and run the  Kinematic Interface
  * \param nh ROS node handle to use
  * \param side string dictating which arm interface to construct
  * \return bool true indicating successful initialization
  */
bool init(ros::NodeHandle& nh, std::string side);

struct Kinematics
{
  KDL::Chain chain;
  std::vector<std::string> joint_names;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver;
  std::unique_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver;
  std::unique_ptr<KDL::ChainIdSolver_RNE>         gravity_solver;
  std::unique_ptr<sns_ik::SNS_IK>                     ik_solver;
  /*std::unique_ptr<KDL::ChainFDSolverTau>           fk_eff_solver; TODO(imcmahon)*/
};
private:
std::string side_, root_name_, tip_name_, camera_name_, gravity_tip_name_;
urdf::Model robot_model_;
intera_core_msgs::JointLimits joint_limits_;
KDL::Tree tree_;
std::map<std::string, double> acceleration_map_;
std::map<std::string, Kinematics> kinematic_chain_map_;

realtime_tools::RealtimeBox< std::shared_ptr<const intera_core_msgs::JointCommand> > joint_command_buffer_;
realtime_tools::RealtimeBox< std::shared_ptr<const sensor_msgs::JointState> > joint_state_buffer_;

ros::Subscriber joint_command_sub_;
ros::Subscriber joint_state_sub_;

ros::Publisher joint_limits_pub_;
ros::Publisher endpoint_state_pub_;
ros::Publisher tip_state_pub_;
long endpoint_state_seq_;
ros::Publisher gravity_torques_pub_;
long gravity_torques_seq_;

ros::ServiceServer ik_service_;
ros::ServiceServer fk_service_;

ros::Timer update_timer_;

/* Method to be invoked at a regular interval for publishing states
 */
void update(const ros::TimerEvent& e);

/* Method to retrieve and populate joint limits from URDF and parameters
 */
intera_core_msgs::JointLimits retrieveJointLimits();


/* Method create a new kinematic chain starting at "base" and ending at "tip_name"
 * @returns true if the new chain was added to the kinematic chains map
 */
bool createKinematicChain(std::string tip_name);

void jointCommandCallback(const intera_core_msgs::JointCommandConstPtr& msg);

/* Callback to capture and store the current joint states of the robot
 */
void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);

/* Method to publish the Gravity Compensation Torques
 */
void publishGravityTorques(); // TODO(imcmahon): publish gravity

/* Method to publish the endpoint state message
 */
void publishEndpointState();

/* Method to parse required parameters from the Param Server
 * @returns true if all parameters found and parsed
 */
bool parseParams(const ros::NodeHandle& nh);

/* Method to service the Forward Kinematics request of any
 * kinematic chain starting at the "base"
 * @returns true to conform to ROS Service signature
 */
bool servicePositionFK(intera_core_msgs::SolvePositionFK::Request& req,
                       intera_core_msgs::SolvePositionFK::Response& res);

/* Method to service the Inverse Kinematics request of any
 * kinematic chain starting at the "base"
 * @returns true to conform to ROS Service signature
 */
bool servicePositionIK(intera_core_msgs::SolvePositionIK::Request& req,
                       intera_core_msgs::SolvePositionIK::Response& res);

/* Method to calculate the position FK for the required joint configuration in rad
 * with the result stored in geometry_msgs::Pose
 *  @returns true if successful
 */
bool computePositionFK(const Kinematics& kin, const KDL::JntArray& jnt_pos, geometry_msgs::Pose& result);

/* Method to calculate the position IK for the required geometry_msg::Pose
 * with the result stored in KDL::JntArray
 *  @returns true if successful
 */
bool computePositionIK(const Kinematics& kin, const geometry_msgs::Pose& cart_pose,
                       const KDL::JntArray& jnt_nullspace_bias,
                       const KDL::JntArray& jnt_seed, KDL::JntArray& result);


/* Method to calculate the velocity FK for the required joint velocities in rad/sec
 * with the result stored in geometry_msgs::Twist
 * @returns true if successful
 */
bool computeVelocityFK(const Kinematics& kin, const KDL::JntArrayVel& jnt_vel, geometry_msgs::Twist& result);

/* Method to calculate the gravity+coriolis+inertia torques for the required joint positions in rad and
 * joint velocities in rad/sec with the result stored in the provided KDL JointArray
 * @returns true if successful
 */
bool computeGravity(const Kinematics& kin, const KDL::JntArray& jnt_pos,
                    const KDL::JntArray& jnt_vel, const KDL::JntArray& jnt_accel,
                    KDL::JntArray& jnt_torques);

/* Method to break down a JointState message object into the corresponding
 * KDL position, velocity, and effort Joint Arrays
 */
void jointStateToKDL(const sensor_msgs::JointState& joint_configuration, const Kinematics& kin,
                     KDL::JntArray& jnt_pos, KDL::JntArray& jnt_vel, KDL::JntArray& jnt_eff);

/* Method to break down a JointState message object into the corresponding
 * the KDL position Joint Array
 */
void jointStatePositionToKDL(const sensor_msgs::JointState& joint_configuration,
                             const Kinematics& kin, KDL::JntArray& jnt_pos)
{
  KDL::JntArray jnt_vel, jnt_eff;
  jointStateToKDL(joint_configuration, kin, jnt_pos, jnt_vel, jnt_eff);
};

/* Method to break down a JointCommand message object into the corresponding
 * the SEAJointState message (aka gravity_compensation_torques)
 */
void jointCommandToGravityMsg(const std::vector<std::string>& joint_names,
                              const intera_core_msgs::JointCommand& command_msg,
                              intera_core_msgs::SEAJointState& gravity_msg);

};
}  // namespace sawyer_gazebo
#endif  // SAWYER_GAZEBO_ARM_KINEMATICS_INTERFACE_H
