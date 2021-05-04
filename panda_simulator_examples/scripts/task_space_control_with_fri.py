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
# @package: panda_siimulator_examples
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
    This is a demo showing task-space control on the 
    simulator robot using the Franka ROS Interface 
    (https://github.com/justagist/franka_ros_interface). 

    ( A more unified package called panda_robot is 
    available which simplifies and combines all the 
    functionalities of Franka ROS Interface and 
    provides a simpler and intuitive python API.
    https://github.com/justagist/panda_robot )

    The task-space force for the desired pose is computed 
    using a simple PD law, and the corresponding
    joint torques are computed and sent to the robot.

    USAGE:
    After launching the simulator (panda_world.launch),
    run this demo using the command:
        
        roslaunch panda_simulator_examples demo_task_space_control.launch --use_fri:=true

"""

import copy
import rospy
import threading
import quaternion
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from franka_interface import ArmInterface

from rviz_markers import RvizMarkers

# --------- Modify as required ------------
# Task-space controller parameters
# stiffness gains
P_pos = 50.
P_ori = 25.
# damping gains
D_pos = 10.
D_ori = 1.
# -----------------------------------------
publish_rate = 100

JACOBIAN = None
CARTESIAN_POSE = None
CARTESIAN_VEL = None

destination_marker = RvizMarkers()


def quatdiff_in_euler(quat_curr, quat_des):
    """
        Compute difference between quaternions and return 
        Euler angles as difference
    """
    curr_mat = quaternion.as_rotation_matrix(quat_curr)
    des_mat = quaternion.as_rotation_matrix(quat_des)
    rel_mat = des_mat.T.dot(curr_mat)
    rel_quat = quaternion.from_rotation_matrix(rel_mat)
    vec = quaternion.as_float_array(rel_quat)[1:]
    if rel_quat.w < 0.0:
        vec = -vec
        
    return -des_mat.dot(vec)

def control_thread(rate):
    """
        Actual control loop. Uses goal pose from the feedback thread
        and current robot states from the subscribed messages to compute
        task-space force, and then the corresponding joint torques.
    """
    while not rospy.is_shutdown():
        error = 100.
        while error > 0.005:
            # when using the panda_robot interface, the next 2 lines can be simplified 
            # to: `curr_pos, curr_ori = panda.ee_pose()`
            curr_pos = robot.endpoint_pose()['position']
            curr_ori = np.asarray(robot.endpoint_pose()['orientation'])

            delta_pos = (goal_pos - curr_pos).reshape([3,1])
            delta_ori = quatdiff_in_euler(curr_ori, goal_ori).reshape([3,1])

            # when using the panda_robot interface, the next 2 lines can be simplified 
            # to: `curr_vel, curr_omg = panda.ee_velocity()`
            curr_vel = robot.endpoint_velocity()['linear'].reshape([3,1])
            curr_omg = robot.endpoint_velocity()['angular'].reshape([3,1])

            # Desired task-space force using PD law
            F = np.vstack([P_pos*(delta_pos), P_ori*(delta_ori)]) - \
                np.vstack([D_pos*(curr_vel), D_ori*(curr_omg)])

            error = np.linalg.norm(delta_pos) + np.linalg.norm(delta_ori)
            
            # panda_robot equivalent: panda.jacobian(angles[optional]) or panda.zero_jacobian()
            J = robot.zero_jacobian()
            
            # joint torques to be commanded
            tau = np.dot(J.T,F)

            # command robot using joint torques
            # panda_robot equivalent: panda.exec_torque_cmd(tau)
            robot.set_joint_torques(dict(list(zip(robot.joint_names(), tau))))
            
            rate.sleep()

def process_feedback(feedback):
    """
    InteractiveMarker callback function. Update target pose.
    """
    global goal_pos, goal_ori

    if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        p = feedback.pose.position
        q = feedback.pose.orientation
        goal_pos = np.array([p.x,p.y,p.z])
        goal_ori = np.quaternion(q.w, q.x,q.y,q.z)

def _on_shutdown():
    """
        Clean shutdown controller thread when rosnode dies.
    """
    global ctrl_thread
    if ctrl_thread.is_alive():
        ctrl_thread.join()
    
if __name__ == "__main__":

    # global goal_pos, goal_ori, ctrl_thread

    rospy.init_node("ts_control_sim_only")

    # when using franka_ros_interface, the robot can be controlled through and
    # the robot state is directly accessible with the API
    # If using the panda_robot API, this will be
    # panda = PandaArm()
    robot = ArmInterface()

    # when using the panda_robot interface, the next 2 lines can be simplified 
    # to: `start_pos, start_ori = panda.ee_pose()`
    ee_pose = robot.endpoint_pose()
    start_pos, start_ori = ee_pose['position'], ee_pose['orientation']

    goal_pos, goal_ori = start_pos, start_ori

    # start controller thread
    rospy.on_shutdown(_on_shutdown)
    rate = rospy.Rate(publish_rate)
    ctrl_thread = threading.Thread(target=control_thread, args = [rate])
    ctrl_thread.start()

    # ------------------------------------------------------------------------------------
    server = InteractiveMarkerServer("basic_control")

    position = Point( start_pos[0], start_pos[1], start_pos[2])
    marker = destination_marker.makeMarker( False, InteractiveMarkerControl.MOVE_ROTATE_3D, \
                                        position, quaternion.as_float_array(start_ori), True)
    server.insert(marker, process_feedback)
    
    server.applyChanges()

    rospy.spin()    
    # ------------------------------------------------------------------------------------