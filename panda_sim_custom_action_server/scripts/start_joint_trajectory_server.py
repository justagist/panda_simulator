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
# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# **************************************************************************/

"""
Emulating the joint trajectory controller server using the SDK.
"""

import argparse
import importlib

import rospy
from dynamic_reconfigure.server import Server

from panda_sim_custom_action_server import (
    JointTrajectoryActionServer,
)

import panda_sim_custom_action_server.cfg.PandaPositionFFJointTrajectoryActionServerConfig as ActionServerConfig

def start_server(rate):
    rospy.loginfo("Initializing node... ")
    rospy.init_node("panda_joint_trajectory_action_server")

    rospy.loginfo("Initializing joint trajectory action server...")
    
    cfg = ActionServerConfig
    dyn_cfg_srv = Server(cfg, lambda config, level: config)
    jtas = []
    
    jtas.append(JointTrajectoryActionServer(dyn_cfg_srv, rate))


    def cleanup():
        for j in jtas:
            j.clean_shutdown()

    rospy.on_shutdown(cleanup)
    rospy.loginfo("Joint Trajectory Action Server Running. Ctrl-c to quit")
    rospy.spin()


def main():

    arg_fmt = argparse.ArgumentDefaultsHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)

    parser.add_argument(
        "-r", "--rate", dest="rate", default=100.0,
        type=float, help="trajectory control rate (Hz)"
    )

    args = parser.parse_args(rospy.myargv()[1:])
    start_server(args.rate)


if __name__ == "__main__":
    main()
