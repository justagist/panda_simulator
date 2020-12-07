#! /usr/bin/env python
import copy
from copy import deepcopy
import rospy
import threading
import quaternion
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import *
#from interactive_markers.interactive_marker_server import *
from franka_interface import ArmInterface
#import pytransform3d.rotations

#from rviz_markers import RvizMarkers
import matplotlib.pyplot as plt

np.set_printoptions(precision=3)

# --------- Tunable control parameters ------------
K_Pt = 50*np.array([[4, 0, 0],
                    [0, 1, 0],
                    [0 ,0 ,0.5]])
K_Po =50*np.identity(3)
#K_d = np.identity(6)*1.5#*2

K_d = np.array([[1.3, 0,  0,  0,  0,  0],
                [0, 1,  0,  0,  0,  0],
                [0, 0,  6,  0,  0,  0],
                [0, 0,  0,  1,  0,  0],
                [0, 0,  0,  0,  1,  0],
                [0, 0,  0,  0,  0,  1]])

K_m = 1*np.identity(6) #not used in simplified impedance controller

starting_position = {'panda_joint1': -0.132, 'panda_joint2': 0.060, 'panda_joint3': 0.131, 'panda_joint4': -2.362, 'panda_joint5': 0.012, 'panda_joint6': 2.381, 'panda_joint7': 0.748}


# --------------------  Helper functions ---------------------


def get_K_Pt_dot(R_d,K_pt,R_e):
    return np.array([0.5*np.linalg.multi_dot([R_d,K_pt,R_d.T])+0.5*np.linalg.multi_dot([R_e,K_pt,R_e.T])])

def get_K_Pt_ddot(p_d,R_d,K_pt):
    return np.array([0.5*np.linalg.multi_dot([skew(p_d-robot.endpoint_pose()['position']),R_d,K_pt,R_d.T])])

def get_K_Po_dot(quat_n,quat_e,R_e,K_po): 
    return np.array([2*np.linalg.multi_dot([E_quat(quat_n,quat_e).T,R_e,K_po,R_e.T])])

def get_h_delta(K_pt_dot,K_pt_ddot,p_delta,K_po_dot,quat_e):
    f_delta_t = np.array([np.dot(K_pt_dot,p_delta)])
    m_delta_t = np.array([np.dot(K_pt_ddot,p_delta)])
    null = np.zeros((3,1))
    m_delta_o = np.array([np.dot(K_po_dot,quat_e)])
    
    return np.array([np.append(f_delta_t.T,m_delta_t.T)]).T + np.array([np.append(null.T,m_delta_o.T)]).T

def skew(vector):
    return np.array([[0, -vector[2], vector[1]], 
                     [vector[2], 0, -vector[0]], 
                     [-vector[1], vector[0], 0]])

def from_three_to_six_dim(matrix):
    return np.block([[matrix,np.zeros((3,3))],[np.zeros((3,3)),matrix]])

def E_quat(quat_n,quat_e): 
    return np.dot(quat_n,np.identity(3))-skew(quat_e)

def get_joint_velocities():
    return np.array([robot.joint_velocity(robot.joint_names()[0]),robot.joint_velocity(robot.joint_names()[1]),robot.joint_velocity(robot.joint_names()[2]),robot.joint_velocity(robot.joint_names()[3]),robot.joint_velocity(robot.joint_names()[4]),robot.joint_velocity(robot.joint_names()[5]),robot.joint_velocity(robot.joint_names()[6])])
    #return np.array(joint_velocities.reshape((7,1)))

def get_joint_angles():
    return np.array([robot.joint_angle(robot.joint_names()[0]),robot.joint_angle(robot.joint_names()[1]),robot.joint_angle(robot.joint_names()[2]),robot.joint_angle(robot.joint_names()[3]),robot.joint_angle(robot.joint_names()[4]),robot.joint_angle(robot.joint_names()[5]),robot.joint_angle(robot.joint_names()[6])])

def get_endpoint_velocities():
    return np.append(robot.endpoint_velocity()['linear'],robot.endpoint_velocity()['angular']).reshape((6,1))

"""
def _on_shutdown():
    
       # Clean shutdown controller thread when rosnode dies.
    
    global ctrl_thread
    if ctrl_thread.is_alive():
        ctrl_thread.join()
"""
def impedance_control(rate,K_Pt,K_Po,K_m,K_d):
    print('activating impedance control')

    # --------- desired state ------------
    a_d_e = np.zeros((6,1)) # desired acceleration relative to fram e
    v_d_e = np.zeros((6,1))#.reshape((6,1))
    pos_d = np.array([0.5,0,0.05]) # Randomly chosen desired position
    Rot_d = np.identity(3)

    #   Desired=Current state
    pos_d = robot.endpoint_pose()['position'] 
    Rot_d = robot.endpoint_pose()['orientation_R']

    force_sensor_offset = np.append(np.zeros(3),np.zeros(3))

    #max_x,max_y,max_z = 0,0,0

    max_num_it = 1411*6
    sensor_readings = np.zeros((6,max_num_it))
    pose_error = np.zeros((6,max_num_it))
    #virtual_wrench_list = np.zeros((6,max_num_it))
    for _iterations in range(max_num_it):

        #   --------- updating parameters ---------
        J = np.array(robot.zero_jacobian())
        #J_weighted_psudu_inv = np.array(np.linalg.multi_dot([np.linalg.inv(robot.joint_inertia_matrix()),J.T,np.linalg.inv(np.linalg.multi_dot([J,np.linalg.inv(robot.joint_inertia_matrix()),J.T]))]))

        #J_dot = np.zeros((6,7))#undefined

        if _iterations == 10: #sensor in sim is buggy on the first iterations
            force_sensor_offset = np.append(robot.endpoint_effort()['force'],robot.endpoint_effort()['torque'])
        h_e = np.append(robot.endpoint_effort()['force'],robot.endpoint_effort()['torque'])-force_sensor_offset #external wrench 

        #virtual_wrench_list[:,_iterations]=virtual_wrench # for plotting
        sensor_readings[:,_iterations]=h_e # for plotting 

        Rot_e = robot.endpoint_pose()['orientation_R']
        Rot_e_bigdim = from_three_to_six_dim(Rot_e)
        Rot_e_dot = np.dot(skew(robot.endpoint_velocity()['angular']),Rot_e) #not a 100 % sure about this one
        Rot_e_dot_bigdim = from_three_to_six_dim(Rot_e_dot)
        
        quat = quaternion.from_rotation_matrix(np.dot(Rot_e.T,Rot_d)) #orientational displacement represented as a unit quaternion
        #quat = robot.endpoint_pose()['orientation']
        quat_e_e = np.array([quat.x,quat.y,quat.z]) # vector part of the unit quaternion in the frame of the end effector
        quat_e = np.dot(Rot_e.T,quat_e_e) # ... in the base frame
        quat_n = quat.w
        
        p_delta = pos_d-robot.endpoint_pose()['position']

        pose_error[:,_iterations]=np.append(p_delta,quat_e)
        K_Pt_dot = get_K_Pt_dot(Rot_d,K_Pt,Rot_e)
        K_Pt_ddot = get_K_Pt_ddot(pos_d,Rot_d,K_Pt)
        K_Po_dot = get_K_Po_dot(quat_n,quat_e,Rot_e,K_Po)

        h_delta_e = np.array(np.dot(Rot_e_bigdim,get_h_delta(K_Pt_dot,K_Pt_ddot,p_delta,K_Po_dot,quat_e)))
        h_e_e = np.array(np.dot(Rot_e_bigdim,h_e))

        delta_v_de_eframe = v_d_e-np.dot(Rot_e_bigdim,get_endpoint_velocities())
        delta_v_de = np.dot(Rot_e_bigdim.T,delta_v_de_eframe)

        #alpha_e = a_d_e + np.dot(np.linalg.inv(K_m),(np.dot(K_d,v_d_e-np.dot(Rot_e_bigdim,get_endpoint_velocities()))+h_delta_e-h_e_e))
        #alpha = np.dot(Rot_e_bigdim.T,alpha_e)+np.dot(Rot_e_dot_bigdim.T,np.dot(Rot_e_bigdim,get_endpoint_velocities()))

        #j_dot_j_inv = np.dot(J_dot,J_weighted_psudu_inv)
        
        # --------------------  Setting torque ---------------------

        #   parameters
        cartesian_inertia = np.linalg.inv(np.linalg.multi_dot([J,np.linalg.inv(robot.joint_inertia_matrix()),J.T]))
        #print(np.shape(np.linalg.multi_dot([J_weighted_psudu_inv.T,J_weighted_psudu_inv,np.array([robot.coriolis_comp()]*6)])))
   
        #coriolis_wrench = np.linalg.multi_dot([J_weighted_psudu_inv.T,J_weighted_psudu_inv,np.array([robot.coriolis_comp()]*6)]) # - np.dot(cartesian_inertia,j_dot_j_inv) #problematic part!!!!!
        #gravitational_wrench = np.array(np.dot(J_weighted_psudu_inv.T,robot.gravity_comp()))

        #   calculating and setting torque
        
        a_d = np.dot(Rot_e_bigdim.T ,a_d_e)

        h_c = np.dot(cartesian_inertia,a_d) + np.dot(K_d, delta_v_de) + np.dot(Rot_e_bigdim.T ,h_delta_e)  #+ np.array(virtual_wrench.reshape((6,1)))
        tau = np.dot(J.T,h_c).reshape((7,1)) + np.array(robot.coriolis_comp().reshape((7,1))) #+ np.array(robot.gravity_comp().reshape((7,1)))
        total_torque = tau


        robot.set_joint_torques(dict(list(zip(robot.joint_names(),total_torque))))

        rate.sleep()
        
        if _iterations % 10 == 0:
            #print(_iterations,':    deviation fom desired position = ',p_delta)
            print(_iterations,':    FT sensor: ',sensor_readings[:,_iterations])#h_e.T)
            print('')
        """
        #evaluation
        if abs(p_delta[0])>max_x:
            max_x = abs(p_delta[0])
        if abs(p_delta[1])>max_y:
            max_y = abs(p_delta[1])
        if abs(p_delta[2])>max_z:
            max_z = abs(p_delta[2])
    
    print('Just exited the control-loop')
    print('maximum deviation from equalibrium:')
    print(' x: ',max_x)
    print(' y: ',max_y)
    print(' z: ',max_z)
    """
    #print('joints:  ',get_joint_angles())
    #print('format:  ',robot._neutral_pose_joints)

    plt.subplot(121)
    plt.title("Sensed external wrench")
    plt.plot(sensor_readings[0,10:], label="force x [N]")
    plt.plot(sensor_readings[1,10:], label="force y [N]")
    plt.plot(sensor_readings[2,10:], label="force z [N]")
    plt.plot(sensor_readings[3,10:], label="torque x [Nm]")
    plt.plot(sensor_readings[4,10:], label="torque y [Nm]")
    plt.plot(sensor_readings[5,10:], label="torque z [Nm]")
    plt.xlabel("number of iterations (adding up to 10 seconds)")
    plt.legend()
    plt.subplot(122)
    plt.title("Deviations from desired pose")
    plt.plot(pose_error[0,10:], label = "deviation x [m]")
    plt.plot(pose_error[1,10:], label = "deviation y [m]")
    plt.plot(pose_error[2,10:], label = "deviation z [m]")
    plt.plot(pose_error[3,10:], label = "quaternion x")
    plt.plot(pose_error[4,10:], label = "quaternion y")
    plt.plot(pose_error[5,10:], label = "quaternion z")
    plt.xlabel("number of iterations (adding up to 10 seconds)")
    plt.legend()
    plt.show()
    
if __name__ == "__main__":

    rospy.init_node("ts_control_sim_only")

    #rospy.on_shutdown(_on_shutdown)
    publish_rate = 500
    rate = rospy.Rate(publish_rate)

    robot = ArmInterface()
    print('Moving to starting position')
    #robot.move_to_joint_positions(starting_position)
    robot.move_to_neutral()
    
    

    impedance_control(rate,K_Pt,K_Po,K_m,K_d)





