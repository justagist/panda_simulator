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
# @package: panda_joint_trajectory_action
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

import rospy

from panda_sim_custom_action_server import GripperActionServer

def start_server():
    rospy.loginfo("Initializing node... ")
    rospy.init_node("panda_gripper_action_server")
    rospy.loginfo("Initializing gripper action server...")

    GripperActionServer()
    rospy.spin()
    rospy.loginfo("Gripper action server running...")


def main():
    start_server()


if __name__ == "__main__":
    main()
