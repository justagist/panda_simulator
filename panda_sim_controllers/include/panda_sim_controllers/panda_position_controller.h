/***************************************************************************
* Adapted from panda_position_controller.h (sawyer_simulator package)

*
* @package: panda_sim_controllers
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

#ifndef PANDA_POSITION_CONTROLLER_H
#define PANDA_POSITION_CONTROLLER_H

#include <mutex>

#include <panda_sim_controllers/joint_array_controller.h>
#include <franka_core_msgs/JointCommand.h>
#include <franka_core_msgs/JointControllerStates.h>
#include <control_msgs/JointControllerState.h>
#include <panda_sim_controllers/panda_joint_position_controller.h>
#include <ros/node_handle.h>
#include <boost/thread.hpp>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_box.h>
#include <panda_hardware_interface/shared_joint_interface.h>


namespace panda_sim_controllers
{
  class PandaPositionController : public panda_sim_controllers::JointArrayController<panda_effort_controllers::JointPositionController>
  {
  public:
    virtual ~PandaPositionController() { sub_joint_command_.shutdown(); t_.join(); }
    virtual bool init(panda_hardware_interface::SharedJointInterface* hw, ros::NodeHandle &n) override;
    void setCommands();

  private:
    // mutex for re-entrant calls to modeCommandCallback
    std::mutex mtx_;
    typedef std::unique_ptr<std::vector<Command>> CommandsPtr;
    ros::Subscriber sub_joint_command_;
    int lastMode;

    boost::thread t_;

    boost::scoped_ptr<realtime_tools::RealtimePublisher<franka_core_msgs::JointControllerStates>> controller_states_publisher_ ;

    ros::Subscriber sub_speed_ratio_;
    ros::Subscriber sub_joint_ctrl_gains_;

    realtime_tools::RealtimeBox< std::shared_ptr<const std_msgs::Float64> > speed_ratio_buffer_;

  /** \brief The callback function to set speed ratio
   *
   * \param msg Speed ratio value.
   */
    void speedRatioCallback(const std_msgs::Float64 msg);

  /** \brief The callback function to set joint commands
   *
   * \param msg joint commands (position).
   */
    void jointCommandCB(const franka_core_msgs::JointCommandConstPtr& msg);

  /** \brief The callback function to set pid controller gain for each joint
   *
   * \param msg joint gains using the franka_core_msgs::JointControllerStates msg.
   */
    void jointCtrlGainsCB(const franka_core_msgs::JointControllerStatesConstPtr& msg);

    void publishControllerState();

    CommandsPtr cmdTrajectoryMode(const franka_core_msgs::JointCommandConstPtr& msg);
    CommandsPtr cmdPositionMode(const franka_core_msgs::JointCommandConstPtr& msg);

  };
}

#endif
