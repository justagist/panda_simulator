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
#ifndef _SAWYER_GAZEBO___ASSEMBLY_INTERFACE_H_
#define _SAWYER_GAZEBO___ASSEMBLY_INTERFACE_H_

#include <ros/ros.h>
#include <realtime_tools/realtime_box.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <intera_core_msgs/AssemblyState.h>


namespace sawyer_gazebo {
  class AssemblyInterface
  {
  public:
    void init(ros::NodeHandle& nh);
  private:
    bool is_enabled_;
    bool is_stopped_;
    std_msgs::Bool sim_estop_;
    ros::Publisher assembly_state_pub_;
    ros::Publisher assembly_sim_estop_pub_;
    ros::Subscriber assembly_enable_sub_;
    ros::Subscriber assembly_stop_sub_;
    ros::Subscriber assembly_reset_sub_;
    ros::Timer assembly_state_timer_;
    realtime_tools::RealtimeBox< std::shared_ptr<const intera_core_msgs::AssemblyState> > assembly_state_buffer_;

    /**
     * Method to stop the robot and capture the source of the stop
     */

    void update(const ros::TimerEvent& e);
    void callbackEnable(const std_msgs::Bool &msg);
    void callbackStop(const std_msgs::Empty &msg);
    void callbackReset(const std_msgs::Empty &msg);

  };
}
#endif // #ifndef _SAWYER_GAZEBO___ASSEMBLY_INTERFACE_H_
