#!/bin/sh
''':'
if [ "$ROS_PYTHON_VERSION" = "3" ]; then
  exec python3 "$0" "$@"
else
  exec python2 "$0" "$@"
fi
'''

# /***************************************************************************

#
# @package: panda_sim_custom_action_server
# @metapackage: panda_simulator
# @author: Saif Sidhik <sxs1412@bham.ac.uk>
#

# **************************************************************************/

# /***************************************************************************
# Copyright (c) 2019-2021, Saif Sidhik

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# **************************************************************************/

"""
Franka Gripper Action Server Emulator using Franka ROS Interface SDK
"""
from math import fabs

import rospy

import actionlib

from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandFeedback,
    GripperCommandResult,
)

import franka_interface

class GripperActionServer(object):
    def __init__(self):
        self._ns = 'franka_gripper/gripper_action'

        self._gripper = franka_interface.GripperInterface(gripper_joint_names = ['panda_finger_joint1', 'panda_finger_joint2'])

        # Action Server
        self._server = actionlib.SimpleActionServer(
            self._ns,
            GripperCommandAction,
            execute_cb=self._on_gripper_action,
            auto_start=False)
        self._action_name = "GripperActionServer"
        self._server.start()

        # Action Feedback/Result
        self._fdbk = GripperCommandFeedback()
        self._result = GripperCommandResult()

        # Initialize Parameters
        self._timeout = 5.0

    def _update_feedback(self, position):
        self._fdbk.position = self._gripper.joint_position('panda_finger_joint1')
        self._fdbk.effort = self._gripper.joint_effort('panda_finger_joint1')
        self._fdbk.stalled = False
        self._fdbk.reached_goal = (fabs(self._fdbk.position -
                                        position) <= 0.05)
        self._result = self._fdbk
        self._server.publish_feedback(self._fdbk)

    def _command_gripper(self, position, force = 0.001, speed = None, wait = False):
        return self._gripper.move_joints(2*position, speed = speed, wait_for_result = wait)

    def _check_state(self, position):
        return fabs(self._gripper.joint_position('panda_finger_joint1') - position) <= 0.01

    def _on_gripper_action(self, goal):
        # Store position and effort from call
        # Position to 0:100 == close:open
        position = goal.command.position
        effort = goal.command.max_effort

        # Reset feedback/result
        self._update_feedback(position)

        # 20 Hz gripper state rate
        control_rate = rospy.Rate(20.0)

        # Record start time
        start_time = rospy.get_time()

        # Set the moving_force/vacuum_threshold based on max_effort provided
        if fabs(effort) < 0.0001:
            effort = None

        def now_from_start(start):
            return rospy.get_time() - start

        # Continue commanding goal until success or timeout
        while ((now_from_start(start_time) < self._timeout or
               self._timeout < 0.0) and not rospy.is_shutdown()):
            if self._server.is_preempt_requested():
                self._gripper.stop_action()
                rospy.loginfo("%s: Gripper Action Preempted" %
                              (self._action_name,))
                self._server.set_preempted(self._result)
                return
            self._update_feedback(position)
            if self._check_state(position):
                self._server.set_succeeded(self._result)
                return
            self._command_gripper(position = position, force = effort, speed = 0.)
            control_rate.sleep()

        # Gripper failed to achieve goal before timeout/shutdown
        self._update_feedback(position)
        result = self._command_gripper(position = position, force = effort, speed = 0., wait = True)

        self._gripper.stop_action()
        if not rospy.is_shutdown():
            rospy.logerr("%s: Gripper Command Not Achieved in Allotted Time" %
                         (self._action_name,))
        self._update_feedback(position)
        self._server.set_aborted(self._result)
