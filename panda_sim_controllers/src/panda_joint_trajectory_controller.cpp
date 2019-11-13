
// Pluginlib
#include <pluginlib/class_list_macros.hpp>

// Project
#include <trajectory_interface/quintic_spline_segment.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>

namespace panda_sim_controllers
{
  /**
   * \brief Joint trajectory controller that represents trajectory segments as <b>quintic splines</b> and sends
   * commands to an \b effort interface.
   */
  typedef joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
                                                                 hardware_interface::EffortJointInterface>
          PandaJointTrajectoryController;
}

PLUGINLIB_EXPORT_CLASS(panda_sim_controllers::PandaJointTrajectoryController,   controller_interface::ControllerBase)