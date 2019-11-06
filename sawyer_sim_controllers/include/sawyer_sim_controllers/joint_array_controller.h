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

#include <map>
#include <memory>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <sawyer_hardware_interface/shared_joint_interface.h>

#define JOINT_ARRAY_CONTROLLER_NAME "joint_array_controller"
namespace sawyer_sim_controllers {
  template <typename T>
  using JointControllerMap = typename std::map<std::string, std::shared_ptr<T> >;

  template <typename T>  // T = Joint(_)Controller
  class JointArrayController : public controller_interface::Controller<sawyer_hardware_interface::SharedJointInterface>
  {
  public:
    JointArrayController() {};
    ~JointArrayController() {};

    /**
     * Store joint commands in a generic format that is compatible with JointPositionController
     */
    struct Command {
      double position_;
      double velocity_;
      double effort_;
      bool has_velocity_;
      std::string name_;
    };

    // gazebo plugin functions
   // bool init(hardware_interface::EffortJointInterface* robot, ros::NodeHandle& nh);
   // void starting(const ros::Time& time);
   // void stopping(const ros::Time& time);
   // void update(const ros::Time& time, const ros::Duration& period);

    size_t n_joints_;
    bool new_command_;
    std::string control_mode_name_;

   // virtual void setCommands(std::map<std::string, Command> cmds);

    realtime_tools::RealtimeBuffer<std::vector<Command> > command_buffer_;
    JointControllerMap<T> controllers_;


    //template <typename T>  // T = Joint(_)Controller
    virtual bool init(sawyer_hardware_interface::SharedJointInterface* robot, ros::NodeHandle& nh)
    {
      return init(robot, nh, "");
    }

    virtual bool init(sawyer_hardware_interface::SharedJointInterface* robot, ros::NodeHandle& nh, const std::string& ctrl_type)
    {
      return _initJointsControls(robot, nh, ctrl_type);
    }

    bool _initJointsControls(sawyer_hardware_interface::SharedJointInterface* robot, ros::NodeHandle& nh, const std::string& ctrl_type)
    {
      // Get joint sub-controllers
      XmlRpc::XmlRpcValue xml_struct;
      if (!nh.getParam("joints", xml_struct))
      {
        ROS_ERROR_NAMED(JOINT_ARRAY_CONTROLLER_NAME, "No 'joints' parameter in controller (namespace '%s')", nh.getNamespace().c_str());
        return false;
      }

      // Make sure it's a struct
      if (xml_struct.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_ERROR_NAMED(JOINT_ARRAY_CONTROLLER_NAME, "The 'joints' parameter is not a struct (namespace '%s')", nh.getNamespace().c_str());
        return false;
      }

      // Get number of joints
      n_joints_ = xml_struct.size();
      control_mode_name_ = nh.getNamespace();
      ROS_INFO_STREAM_NAMED(JOINT_ARRAY_CONTROLLER_NAME, "Initializing JointArrayController(type:" << control_mode_name_ <<") with " << n_joints_ << " joints.");

      int i = 0;  // track the joint id
      for (XmlRpc::XmlRpcValue::iterator joint_it = xml_struct.begin(); joint_it != xml_struct.end(); ++joint_it)
      {
        // Get joint controller
        if (joint_it->second.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
          ROS_ERROR_NAMED(JOINT_ARRAY_CONTROLLER_NAME, "The 'joints/joint_controller' parameter is not a struct (namespace '%s')",
                          nh.getNamespace().c_str());
          return false;
        }

        // Get joint controller name
        std::string joint_controller_name = joint_it->first;

        // Get the joint name
        std::string joint_name = joint_it->second["joint"];

        // Get the joint-namespace nodehandle
        {
          ros::NodeHandle joint_nh(nh, "joints/" + joint_controller_name);
          ROS_INFO_STREAM_NAMED(JOINT_ARRAY_CONTROLLER_NAME, "Loading sub-controller '" << joint_controller_name
                                                                   << "', Namespace: " << joint_nh.getNamespace());
          if (ctrl_type.length() <= 0)
          {
            // This is a main ctrl command
            controllers_[joint_name].reset(new T());
            controllers_[joint_name]->init(robot, joint_nh);
          }
          else
          {
            // Acquire a compounded sub-command handle - we're going to Add (+) our command to the main ctrl cmd
            controllers_[joint_name].reset(new T());
            controllers_[joint_name]->init(robot, joint_nh, ctrl_type);
          }

        }  // end of joint-namespaces

        // increment joint i
        ++i;
      }

      return true;
    }

    //template <typename T>
    void starting(const ros::Time& time) {
      ROS_DEBUG_STREAM_NAMED(JOINT_ARRAY_CONTROLLER_NAME, "STARTING - JointArrayController for '" << control_mode_name_ << "' mode.");

      for (auto it = controllers_.begin(); it != controllers_.end(); it++) {
        it->second->starting(time);
      }
    }

    //template <typename T>
    void stopping(const ros::Time& time) {
      ROS_DEBUG_STREAM_NAMED(JOINT_ARRAY_CONTROLLER_NAME, "Stopping - JointArrayController for '" << control_mode_name_ << "' mode.");

    }

    virtual void setCommands() {};

    //template <typename T>
    void update(const ros::Time& time, const ros::Duration& period) {
      // first check if there are new commands
      if (new_command_) {
        // assume we will succeed at this command
        new_command_ = false;

        // set the new commands for all controllers
        setCommands();
      }

      // always update all controllers
      for (auto it = controllers_.begin(); it != controllers_.end(); it++) {
        it->second->update(time, period);
      }
    }
  };
}
