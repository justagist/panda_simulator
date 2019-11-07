/***************************************************************************
* Adapted from shared_joint_interface.h (sawyer_simulator package)

*
* @package: panda_hardware_interface
* @metapackage: panda_simulator
* @author: Saif Sidhik <sxs1412@bham.ac.uk>
*

**************************************************************************/

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

#ifndef _PANDA_HARDWARE_INTERFACE___SHARED_JOINT_INTERFACE_H_
#define _PANDA_HARDWARE_INTERFACE___SHARED_JOINT_INTERFACE_H_

#include <cassert>
#include <string>
#include <memory>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <panda_hardware_interface/sum_command_interface.h>

namespace panda_hardware_interface
{

/** \brief Replacement for EffortJointInterface that sums cmds from multiple
 *         handles for the same joint.
 *
 * getHandle()
 * Gives out multiple JointHandles (subHandles) for the same joint.
 * Controllers can ask for either a unique subHandle by "subType" or get the
 * shared main subHandle by default.
 *
 * For each joint, a SumJointHandle container can be created/registered, which
 * will manage the multiple (sub) JointHandles for that joint.
 *
 */
class SharedJointInterface : public hardware_interface::EffortJointInterface
{
public:
// todo: should we make a createContainer(&js, cmd*) here so it's more RAII?

  void registerContainer(const SumJointHandlePtr& ctrHandle)
  {
    // using simple std container (not HWResourceMgr)
    _registerContainer(ctrHandle);
  }

  /**
   * \brief Register a new resource.
   * If the resource name already exists, the previously stored resource value will be replaced with \e val.
   * \param handle Resource value. Its type should implement a <tt>std::string getName()</tt> method.
   */
  void _registerContainer(const SumJointHandlePtr& handle)
  {
    ContainerHandlesMap::iterator it = containers_.find(handle->getName());
    if (it == containers_.end())
    {
      containers_.insert(std::make_pair(handle->getName(), handle));
      ROS_DEBUG_STREAM("Added Container '" << handle->getName() << "'");
    }
    else
    {
      ROS_WARN_STREAM("Replacing previously registered handle '" << handle->getName() << "' in '" +
                      hardware_interface::internal::demangledTypeName(*this) + "'.");
      it->second = handle;
    }
  }

  /**
   * \brief Get a resource handle by name.
   * \param name Resource name.
   * \return Resource associated to \e name. If the resource name is not found, an exception is thrown.
   */
  SumJointHandlePtr getContainer(const std::string& name)
  {
    // using simple std container (not HWResourceMgr)
    return _getContainerHandle(name);
  }

  SumJointHandlePtr _getContainerHandle(const std::string& name)
  {
    ContainerHandlesMap::const_iterator it = containers_.find(name);

    if (it == containers_.end())
    {
      ROS_ERROR_STREAM("Could not find Container '" << name << "' in (" << containers_.size() << ") containers. "
        << "Including: ");
      std::vector<std::string> names = getNames();
      for(unsigned int i=0; i < names.size(); i++) {
        ROS_ERROR_STREAM(" * " << names[i]);
      }

      throw hardware_interface::HardwareInterfaceException("HEY LISTEN! Could not find resource '" + name + "' in '" +
        hardware_interface::internal::demangledTypeName(*this) + "'." +
        "Out of (" + std::to_string(containers_.size()) + ") Containers.");
    }
    std::vector<std::string> names = getNames();

    // ROS_WARN_STREAM("HERE!! ");
    // ROS_WARN_STREAM("CONTAINERS SIZE: " << std::to_string(containers_.size()));
    return it->second;
  }

  // ------ override ej-interface : HwRsrcMgr implementation   ------

  hardware_interface::JointHandle getHandle(const std::string& name)  // name=jointName
  {
    return _getHandle(name);
  }

  hardware_interface::JointHandle _getHandle(const std::string& name)  // name=jointName
  {
    try
    {
      SumJointHandlePtr ctrHandle = getContainer(name);

      return getHandle(name, ctrHandle->getDefaultSubType());
    }
    catch(const std::logic_error& e)
    {
      throw hardware_interface::HardwareInterfaceException(e.what());
    }
  }

  hardware_interface::JointHandle getHandle(const std::string& name, const std::string& subType)
  {
    try
    {
      SumJointHandlePtr ctrHandle = getContainer(name);

      hardware_interface::JointHandle out = ctrHandle->getSubHandle(name, subType);

      // If ClaimPolicy type is ClaimResources, the below method claims resources, for DontClaimResources it's a no-op
      hardware_interface::ClaimResources::claim(this, name + '/' + subType);

      return out;
    }
    catch(const std::logic_error& e)
    {
      throw hardware_interface::HardwareInterfaceException(e.what());
    }
  }

  // ------ override ej-interface : ResourceManager impl -------

  void registerHandle(const ResourceHandleType & handle)
  {
    // FIXME: Would be nice if there was a better fitting class relationship
    ROS_ERROR_STREAM("Tried calling underlying registerHandle() on SumCmdsInterface!");
  }

  std::vector<std::string> getNames() const
  {
    // Hmmm.... not sure what the 'proper' behavior here should be - include both types (Containers, CmdHandles)?

    std::vector<std::string> out;
    for(ContainerHandlesMap::const_iterator it = containers_.begin(); it != containers_.end(); ++it)
    {
      out.push_back(it->first);
      it->second->appendSubNames(out);
    }
    return out;
  }

protected:
  typedef std::map<std::string, SumJointHandlePtr> ContainerHandlesMap;

  ContainerHandlesMap containers_;
};


}  // namespace


#endif /* _PANDA_HARDWARE_INTERFACE___SHARED_JOINT_INTERFACE_H_ */
