#! /usr/bin/env python
import copy
from copy import deepcopy
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


# Control parameters
K = 10
D = 2
# -----------------------------------------
# External force

start_ext_force = 50 # The iteration where the external force is activated
end_ext_force = 60

# Constants
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

def false_feedback(interval_start,interval_end, iteration):
    #x_acc = 0 # Could not find a function for it 
    #x_speed = robot.endpoint_velocity()['linear'][0]
    #rel_x_position = robot.endpoint_pose()['position'][0] - eq_position
    if iteration > interval_start and iteration < interval_end :
        #return np.array([M_env*x_acc + B_env*x_speed + K_env*rel_x_position,0,0])
        return np.array([
            0, #Fx
            0, #Fy
            30, #Fz,
            0, #Torque x
            0, #Torque Y
            0 # Torque z
        ])
    else:
        return np.array([0]*6)




def control_thread(rate):
    """
        Actual control loop. Uses goal pose from the feedback thread
        and current robot states from the subscribed messages to compute
        task-space force, and then the corresponding joint torques.
    """
    print('')
    print('')
    print('-------------INSIDE CONTROL_THREAD()-------------')
    print('')
    print('')
    #while not rospy.is_shutdown():
    print('moving to neutral position (default function)')
    robot.move_to_neutral()
    print('at neutral position')
    neutral_pose = np.array([robot._neutral_pose_joints['panda_joint1'],robot._neutral_pose_joints['panda_joint2'],robot._neutral_pose_joints['panda_joint3'],robot._neutral_pose_joints['panda_joint4'],robot._neutral_pose_joints['panda_joint5'],robot._neutral_pose_joints['panda_joint6'],robot._neutral_pose_joints['panda_joint7']])

    tot_num_loops = 2
    for loop in range(tot_num_loops):
        print('Starting loop number',loop+1,' out of ',tot_num_loops)
        #init_history() #"""testing"""
        #receive_user_input()
        #error = 100
        iterations = 0
        tot_num_iterations = 500
        while iterations < tot_num_iterations:

            # panda_robot equivalent: panda.jacobian(angles[optional]) or panda.zero_jacobian()
            J = robot.zero_jacobian()
            
            # joint torques to be commanded
            #tau = np.dot(J.T,F)'
            joint_velocities = np.array([robot.joint_velocity(robot.joint_names()[0]),robot.joint_velocity(robot.joint_names()[1]),robot.joint_velocity(robot.joint_names()[2]),robot.joint_velocity(robot.joint_names()[3]),robot.joint_velocity(robot.joint_names()[4]),robot.joint_velocity(robot.joint_names()[5]),robot.joint_velocity(robot.joint_names()[6])])
            F_ext = false_feedback(start_ext_force,end_ext_force,iterations)
            tau_ext = np.dot(J.T, F_ext)
            #print('tau_ext: ',tau_ext)
            tau = tau_ext + K*(neutral_pose-np.array([0,0,0,0,0,0,0])-robot.joint_ordered_angles())+D*(np.array([0,0,0,0,0,0,0])-joint_velocities) +robot.coriolis_comp()+ robot.gravity_comp()
            #print('Tau: ',tau)
            
             
            #"""
            #print('J:   ',J)
            #print('')
            #print('Tau: ', tau)
            #"""
            # command robot using joint torques
            # panda_robot equivalent: panda.exec_torque_cmd(tau)
            robot.set_joint_torques(dict(list(zip(robot.joint_names(), tau))))
            
            rate.sleep()
            if iterations % 10 ==0:
                #print('At iteration: ',iterations,'. Error is ',error)
                """
                if iterations > start_ext_force and iterations < end_ext_force :
                    print('At iteration: ',iterations, '    External force feedback: ', F_ext)
                    print('')
                else:
                    print('At iteration: ',iterations,' /',tot_num_iterations)
                    print('')
                """
                Rot_e = robot.endpoint_pose()['orientation_R']
                #print(np.block([[Rot_e,np.zeros((3,3))],[np.zeros((3,3)),Rot_e]]))
            iterations +=1

        print('Loop ',loop+1,' ended')

    print('last loop ended. (',loop+1,'/',tot_num_loops,')')
        


def _on_shutdown():
    
       # Clean shutdown controller thread when rosnode dies.
    
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

    #ee_pose = robot.endpoint_pose()
    #start_pos, start_ori = ee_pose['position'], ee_pose['orientation']
    tip_state = robot.tip_states()


    # start controller thread
    #rospy.on_shutdown(_on_shutdown)
    rate = rospy.Rate(publish_rate)
    #print('goal_pos: ',goal_pos)
    control_thread(rate)

    #ctrl_thread = threading.Thread(target=control_thread, args = [rate])
    print("")
    print('')
    #ctrl_thread.start()  