/***************************************************************************
* Adapted from arm_kinematics_interface.h (sawyer_simulator package)

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
#ifndef PANDA_GAZEBO_ARM_KINEMATICS_INTERFACE_H
#define PANDA_GAZEBO_ARM_KINEMATICS_INTERFACE_H

#include <map>
#include <string>

#include <ros/ros.h>
#include <realtime_tools/realtime_box.h>

#include <sensor_msgs/JointState.h>
#include <franka_core_msgs/JointCommand.h>
#include <franka_core_msgs/JointLimits.h>
#include <franka_core_msgs/RobotState.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>

#include <Eigen/Geometry>

#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <urdf/model.h>
#include "kdl/chainfksolver.hpp"

#include <panda_gazebo/kdl_methods.h>

namespace panda_gazebo
{
class ArmKinematicsInterface
{
public:
/** Method to initialize and run the  Kinematic Interface
  * \param nh ROS node handle to use
  * \param side string dictating which arm interface to construct
  * \return bool true indicating successful initialization
  */
bool init(ros::NodeHandle& nh);

struct Kinematics
{
  KDL::Chain chain;
  std::vector<std::string> joint_names;
  
};
private:
std::string root_name_, tip_name_, gravity_tip_name_;
urdf::Model robot_model_;
franka_core_msgs::JointLimits joint_limits_;
KDL::Tree tree_;
std::map<std::string, double> acceleration_map_;
std::map<std::string, Kinematics> kinematic_chain_map_;

realtime_tools::RealtimeBox< std::shared_ptr<const franka_core_msgs::JointCommand> > joint_command_buffer_;
realtime_tools::RealtimeBox< std::shared_ptr<const sensor_msgs::JointState> > joint_state_buffer_;
realtime_tools::RealtimeBox<std::shared_ptr<const geometry_msgs::Wrench>> ft_msg_buffer_;

ros::Subscriber joint_command_sub_;
ros::Subscriber joint_state_sub_;
ros::Subscriber ft_sensor_sub_;

ros::Publisher joint_limits_pub_;
ros::Publisher endpoint_state_pub_;
ros::Publisher robot_state_publisher_;
long endpoint_state_seq_;
long gravity_torques_seq_;

ros::Timer update_timer_;

KDLMethods* kdl_;


  // std::unique_ptr<KDL::ChainFkSolverPos_recursive fk_pos_solver_;

/* Method to be invoked at a regular interval for publishing states
 */
void update(const ros::TimerEvent& e);

/* Method to retrieve and populate joint limits from URDF and parameters
 */
franka_core_msgs::JointLimits retrieveJointLimits();


/* Method create a new kinematic chain starting at "base" and ending at "tip_name"
 * @returns true if the new chain was added to the kinematic chains map
 */
bool createKinematicChain(std::string tip_name);

void jointCommandCallback(const franka_core_msgs::JointCommandConstPtr& msg);

/* Callback to capture and store the current joint states of the robot
 */
void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);

/* Callback to capture and store the current force torque values from wrist sensor
 */
void ftSensorCallback(const geometry_msgs::WrenchStampedConstPtr &msg);

/* Compute Jacobian and end effector velocity and add to Robot State message
 */
void addJacAndVelToMsg(const Kinematics& kin, const KDL::JntArray& jnt_pos,
                       const KDL::JntArray& jnt_vel, franka_core_msgs::RobotState& robot_state);

/* Compute inertia, coriolis and gravity, and add to Robot State message
 */
void addDynamicsToMsg(const Kinematics& kin, const KDL::JntArray& jnt_pos,
                       const KDL::JntArray& jnt_vel, franka_core_msgs::RobotState& robot_state);

/* Method to publish the endpoint state message
 */
void publishEndpointState();

/* Method to publish the robot state message
 */
void publishRobotState();

/* Method to parse required parameters from the Param Server
 * @returns true if all parameters found and parsed
 */
bool parseParams(const ros::NodeHandle& nh);

/* Method to calculate the position FK for the required joint configuration in rad
 * with the result stored in geometry_msgs::Pose
 *  @returns true if successful
 */
bool computePositionFK(const Kinematics& kin, const KDL::JntArray& jnt_pos, geometry_msgs::Pose& result);


/* Method to calculate the velocity FK for the required joint velocities in rad/sec
 * with the result stored in geometry_msgs::Twist
 * @returns true if successful
 */
// bool computeVelocityFK(const Kinematics& kin, const KDL::JntArrayVel& jnt_vel, geometry_msgs::Twist& result);

// bool computeEffortFK(const Kinematics& kin,
//                                              const KDL::JntArray& jnt_pos,
//                                              const KDL::JntArray& jnt_eff,
//                                              geometry_msgs::Wrench& result);

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
 * the gravity torques
 */
void addGravityToMsg(const std::vector<std::string>& joint_names,
                              const franka_core_msgs::JointCommand& command_msg,
                              franka_core_msgs::RobotState& robot_state, std::array<double, 7> &acc);

};
}  // namespace panda_gazebo
#endif  // PANDA_GAZEBO_ARM_KINEMATICS_INTERFACE_H
