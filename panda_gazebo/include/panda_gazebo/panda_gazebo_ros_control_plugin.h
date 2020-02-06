/***************************************************************************
* Adapted from sawyer_gazebo_ros_interface.h (sawyer_simulator package)

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
#ifndef _PANDA_GAZEBO___PANDA_GAZEBO_ROS_CONTROL_PLUGIN_H_
#define _PANDA_GAZEBO___PANDA_GAZEBO_ROS_CONTROL_PLUGIN_H_
// Overload the default plugin
#include <gazebo_ros_control/gazebo_ros_control_plugin.h>
#include <panda_gazebo/arm_controller_interface.h>
#include <panda_gazebo/arm_kinematics_interface.h>


namespace panda_gazebo {

  class PandaGazeboRosControlPlugin : public gazebo_ros_control::GazeboRosControlPlugin
  {
  private:
    // AssemblyInterface assembly_interface_;
    ArmControllerInterface arm_controller_interface_;
    ArmKinematicsInterface arm_kinematics_interface_;
    // HeadInterface head_interface_;

  protected:
    void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);


  };
}
#endif // #ifndef __PANDA_GAZEBO__PANDA_GAZEBO_ROS_CONTROL_PLUGIN_H_
