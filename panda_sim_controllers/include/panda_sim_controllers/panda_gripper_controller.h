/***************************************************************************

*
* @package: panda_sim_controllers
* @metapackage: panda_simulator
* @author: Saif Sidhik <sxs1412@bham.ac.uk>
*

**************************************************************************/

/*********************************************************************
 * Copyright (c) 2019-2020, Saif Sidhik
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

#ifndef PANDA_GRIPPER_CONTROLLER_H_
#define PANDA_GRIPPER_CONTROLLER_H_

#include <mutex>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_box.h>
#include <panda_sim_controllers/panda_joint_position_controller.h>  // used for controlling individual joints

#include <cmath>
#include <string>

#include <actionlib/server/simple_action_server.h>
#include <ros/node_handle.h>

#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>




namespace panda_sim_controllers
{
  class PandaGripperController : public controller_interface::Controller<panda_hardware_interface::SharedJointInterface>
  {


  private:

    ros::NodeHandle nh_;

    // Last Gripper Command Position
    realtime_tools::RealtimeBox< std::shared_ptr<double> > gripper_pos_command_buffer_;
    realtime_tools::RealtimeBox< std::shared_ptr<double> > gripper_speed_command_buffer_;

    actionlib::SimpleActionServer<franka_gripper::HomingAction> homing_action_server;
    actionlib::SimpleActionServer<franka_gripper::MoveAction> move_action_server;
    actionlib::SimpleActionServer<franka_gripper::GraspAction> grasp_action_server;
    actionlib::SimpleActionServer<franka_gripper::StopAction> stop_action_server;

    // Indices for mimic and orignal joints
    int mimic_idx_;
    int main_idx_;

    size_t n_joints_;
    double start_position_;
    double last_position_;
    // An indicator to evalute to true when an unproccessed new command is in the realtime buffer
    bool new_command_;
    size_t update_counter_;
    // franka_core_msgs::IONodeConfiguration config_;

    // Command subscriber
    // ros::Subscriber gripper_command_sub_;
    // // State publisher
    ros::Publisher state_pub_;

    sensor_msgs::JointState joint_state_msg_;
    // // Config publisher
    // ros::Publisher gripper_config_pub_;
    // // Higher level end effector config publisher
    // ros::Publisher end_effector_config_pub_;
    // // Timer to update publishers
    ros::Timer pub_timer_;

    // franka::GripperState gripper_state_;
    std::mutex gripper_state_mutex_;
   /**
    * @brief Regular update for publishing config and state
    * @param event ROS Timer event triggered
    */
    void timerUpdate(const ros::TimerEvent& event);

   /**
    * @brief Retrieves new command and updates both fingers controllers
    */
    void updateCommands();


    // Create an effort-based joint position controller for every joint
    std::vector<
      std::shared_ptr<
        panda_effort_controllers::JointPositionController> > gripper_controllers_;

/**
 * Calls the libfranka move service of the gripper
 *
 * @param[in] gripper A gripper instance to execute the command
 * @param[in] goal A move goal with target width and velocity
 *
 * @return True if command was successful, false otherwise.
 */
void move(const franka_gripper::MoveGoalConstPtr& goal);

/**
 * Calls the libfranka homing service of the gripper
 *
 * @param[in] gripper A gripper instance to execute the command
 *
 * @return True if command was successful, false otherwise.
 */
void homing(const franka_gripper::HomingGoalConstPtr& /*goal*/);

/**
 * Calls the libfranka stop service of the gripper to stop applying force
 *
 * @param[in] gripper A gripper instance to execute the command
 *
 * @return True if command was successful, false otherwise.
 */
void stop(const franka_gripper::StopGoalConstPtr& /*goal*/);

/**
 * Calls the libfranka grasp service of the gripper
 *
 * An object is considered grasped if the distance \f$d\f$ between the gripper fingers satisfies
 * \f$(\text{width} - \text{epsilon_inner}) < d < (\text{width} + \text{epsilon_outer})\f$.
 *
 * @param[in] gripper A gripper instance to execute the command
 * @param[in] goal A grasp goal with target width, epsilon_inner, epsilon_outer, velocity and effort
 * @return True if an object has been grasped, false otherwise.
 */
void grasp(const franka_gripper::GraspGoalConstPtr& goal);

  public:
    PandaGripperController();
    virtual ~PandaGripperController();

    bool init(panda_hardware_interface::SharedJointInterface* robot, ros::NodeHandle &n) ;
    void starting(const ros::Time& time){} ;
    void stopping(const ros::Time& time){} ;
    void update(const ros::Time& time, const ros::Duration& period) ;


  };

} // namespace

#endif /* PANDA_GRIPPER_CONTROLLER_H_ */
