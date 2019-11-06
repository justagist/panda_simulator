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

#ifndef _SAWYER_HARDWARE_INTERFACE___SUM_COMMAND_INTERFACE_H_
#define _SAWYER_HARDWARE_INTERFACE___SUM_COMMAND_INTERFACE_H_

#include <cassert>
#include <string>
#include <memory>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

namespace sawyer_hardware_interface
{


/** \brief Container handle used to manage and sum all the sub-JointHandles that
 *         command a single joint.
 *
 *  Manages multiple "sub" JointHandles for a single joint. Controllers can ask for
 *  either a unique subHandle by "subType" or get the main, shared subHandle by default.
 *
 *  The SumJointHandle will store the individual cmd values for each sub JointHandle and
 *  stores the reference to the actual output &cmd_[] from the robotHwSim ej_interface.
 *  To the individual Controllers, the JointHandle's behave the same as normal.
 *
 *  Then SumJointHandle.updateCommandSum() can be called to sum the sub-Commands and
 *  write the sum to the output &cmd_[].
 */
class SumJointHandle
{
public:

  /**
   * \param js  The JointState handle for this joint
   *            (used for read; from robotHWSim::js_interface_.getHandle())
   * \param cmd A pointer to the storage for this joint's output command effort
   *            (write target of summed sub-handles; same one used by normal ej_interface)
   */
  SumJointHandle(const hardware_interface::JointStateHandle& js, double* cmd)
    : js_(js),
      cmd_(cmd),
      name_(js.getName())
  {
    if (!cmd)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command pointer is null.");
    }
  }

  hardware_interface::JointHandle getDefaultSubHandle(const std::string& jointName)
  {
    return getSubHandle(jointName, "MAIN_SETPOINT_COMMAND");
  }

  std::string getDefaultSubType() {
    return "MAIN_SETPOINT_COMMAND";
  }

  hardware_interface::JointHandle getSubHandle(const std::string& jointName)
  {
    return getSubHandle(jointName, getDefaultSubType());
  }

  // TODO: make create/GetSubHandle() that is for init, and may or may not create new one,
  //       and separate getSubHandle() that will only get existing (safe for update)

  hardware_interface::JointHandle getSubHandle(const std::string& jointName, const std::string& subType)
  {
    SubJointHandleMap::const_iterator it = sub_handles_.find(subType);
    if (it == sub_handles_.end())
    {
      // Create and register subHandle
      sub_commands_[subType] = 0.0;
      sub_handles_[subType] = hardware_interface::JointHandle(js_, &sub_commands_[subType]);

      ROS_DEBUG_STREAM("Creating SubHandle (" << howManySubs() << "): '" << sub_handles_[subType].getName()
        << "'/'" << subType << "'.");
    }
    else
    {
      // SubHandle already exists
      // NB: For better or worse, all the USER cmds will be using same subHandle...
      ROS_DEBUG_STREAM("Reusing SubHandle (" << howManySubs() << "): '" << sub_handles_[subType].getName()
        << "'/'" << subType << "'.");
    }
    return sub_handles_[subType];
  }

  std::string getName() const {
    return name_;
  }

  void updateCommandSum()
  {
    // get and sum the individual commands for each sub interface
    assert(cmd_);
    *cmd_ = getCommand();
  }

  /**
   * \brief Get sum total command
   */
  double getCommand()
  {
    // get and sum the individual commands for each sub interface
    double total = 0.0;

    // Just iterate over the numerical cmd array until we can figure out what's going on w/ "deactivating" cmds
    for (std::map<std::string, double>::const_iterator it = sub_commands_.begin(); it != sub_commands_.end(); ++it)
    {
      total += it->second;
    }

    return total;
  }


  // like getNames() for the subHandles, but add to an existing list (b/c that's how I need it atm)
  void appendSubNames(std::vector<std::string>& namesList)
  {
    for(SubJointHandleMap::const_iterator it = sub_handles_.begin(); it != sub_handles_.end(); ++it)
    {
      namesList.push_back(it->first);
    }
  }

  int howManySubs()
  {
    return sub_handles_.size();
  }

private:
  typedef std::map<std::string, hardware_interface::JointHandle> SubJointHandleMap;

  std::string name_;
  double* cmd_;          // pointer to storage for output command to ej_interface
  std::map<std::string, double> sub_commands_;  // storage for sub-command values

  SubJointHandleMap sub_handles_;

  hardware_interface::JointStateHandle js_;

  // TODO: replace use of *cmd_ w/ writing to a JointHandle from ej_interface
  //       (pass in & store ej_interface.getHandle(j), like jsHandle is done)
};


typedef std::shared_ptr<SumJointHandle> SumJointHandlePtr;


}  // namespace


#endif /*_SAWYER_HARDWARE_INTERFACE___SUM_COMMAND_INTERFACE_H_*/
