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

# --------- Tunable control parameters ------------
K_Pt = 10*np.array([[3,0,0],
                    [0,3,0],
                    [0,0,1]])
K_Po =60*np.identity(3)
K_m = np.identity(6)
K_d = 2*np.identity(6)


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

def get_endpoint_velocities():
    return np.append(robot.endpoint_velocity()['linear'],robot.endpoint_velocity()['angular']).reshape((6,1))

def virtual_external_force(interval_start,interval_end, iteration,f):
    if iteration > interval_start and iteration < interval_end :
        return np.array([
            0, #Fx
            f, #Fy
            0, #Fz,
            0, #Torque x
            0, #Torque Y
            0 # Torque z
        ]).reshape((6,1))
    else:
        return np.zeros((6,1))
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
    pos_d = np.array([0.5,0.5,0.5]) # Randomly chosen desired position
    Rot_d = np.identity(3)

    #   Desired=Current state
    pos_d = robot.endpoint_pose()['position'] 
    Rot_d = robot.endpoint_pose()['orientation_R']

    max_x,max_y,max_z = 0,0,0
    for _iterations in range(400):

        #   --------- updating parameters ---------
        J = np.array(robot.zero_jacobian())
        J_weighted_psudu_inv = np.array(np.linalg.multi_dot([np.linalg.inv(robot.joint_inertia_matrix()),J.T,np.linalg.inv(np.linalg.multi_dot([J,np.linalg.inv(robot.joint_inertia_matrix()),J.T]))]))

        J_dot = np.zeros((6,7))#undefined


        """
        setting force-sensor-readings = virtual external force
        """
        this_virtual_external_force = virtual_external_force(50,250,_iterations,4)
        h_e = np.append(robot.endpoint_effort()['force'],robot.endpoint_effort()['torque']) #external wrench
        #h_e = -this_virtual_external_force
        h_e = np.array(h_e).reshape((6,1))
        Rot_e = robot.endpoint_pose()['orientation_R']
        Rot_e_bigdim = from_three_to_six_dim(Rot_e)
        Rot_e_dot = np.dot(skew(robot.endpoint_velocity()['angular']),Rot_e) #not a 100 % sure about this one
        Rot_e_dot_bigdim = from_three_to_six_dim(Rot_e_dot)
        
        quat = quaternion.from_rotation_matrix(np.dot(Rot_e.T,Rot_d))
        #quat = robot.endpoint_pose()['orientation']
        quat_e_e = np.array([quat.x,quat.y,quat.z])
        quat_e = np.dot(Rot_e.T,quat_e_e)
        quat_n = quat.w
        
        p_delta = pos_d-robot.endpoint_pose()['position']

        K_Pt_dot = get_K_Pt_dot(Rot_d,K_Pt,Rot_e)
        K_Pt_ddot = get_K_Pt_ddot(pos_d,Rot_d,K_Pt)
        K_Po_dot = get_K_Po_dot(quat_n,quat_e,Rot_e,K_Po)

        h_delta_e = np.array(np.dot(Rot_e_bigdim,get_h_delta(K_Pt_dot,K_Pt_ddot,p_delta,K_Po_dot,quat_e)))
        h_e_e = np.array(np.dot(Rot_e_bigdim,h_e))

        alpha_e = a_d_e + np.dot(np.linalg.inv(K_m),(np.dot(K_d,v_d_e-np.dot(Rot_e_bigdim,get_endpoint_velocities()))+h_delta_e-h_e_e))
        alpha = np.dot(Rot_e_bigdim.T,alpha_e)+np.dot(Rot_e_dot_bigdim.T,np.dot(Rot_e_bigdim,get_endpoint_velocities()))

        j_dot_j_inv = np.dot(J_dot,J_weighted_psudu_inv)
        
        # --------------------  Setting torque ---------------------

        #   parameters
        cartesian_inertia = np.linalg.inv(np.linalg.multi_dot([J,np.linalg.inv(robot.joint_inertia_matrix()),J.T]))
        #print(np.shape(np.linalg.multi_dot([J_weighted_psudu_inv.T,J_weighted_psudu_inv,np.array([robot.coriolis_comp()]*6)])))
   
        coriolis_wrench = np.linalg.multi_dot([J_weighted_psudu_inv.T,J_weighted_psudu_inv,np.array([robot.coriolis_comp()]*6)]) # - np.dot(cartesian_inertia,j_dot_j_inv) #problematic part!!!!!
        gravitational_wrench = np.array(np.dot(J_weighted_psudu_inv.T,robot.gravity_comp())) #should gravity and coriolis gave a negative sign? since its called comp(ensation)

        #   calculating and setting torque
        h_c = np.dot(cartesian_inertia,alpha) + np.dot(coriolis_wrench,get_joint_velocities()).reshape((6,1))+h_e
        tau = np.dot(J.T,h_c).reshape((7,1))+np.array(robot.gravity_comp().reshape((7,1)))
        total_torque = tau + np.dot(J.T,this_virtual_external_force)

        """
        print('h_c-test: ',np.shape(np.dot(cartesian_inertia,alpha)),np.shape(np.dot(coriolis_wrench,get_joint_velocities())),np.shape(gravitational_wrench))
        print('h_delta_e',np.shape(h_delta_e))
        print('h_e',np.shape(h_e))
        print('h_e_e',np.shape(h_e_e))
        print('v_delta: ',np.shape(v_d_e-np.dot(Rot_e_bigdim,get_endpoint_velocities())))
        print('hele parantesen etter K_m_inv: ',np.shape(np.dot(K_d,v_d_e-np.dot(Rot_e_bigdim,get_endpoint_velocities()))+h_delta_e-h_e_e))
        print('sum av to 6x1 matriser: ', np.array(h_delta_e-h_e_e))
        print('dim a_d_e: ', np.shape(a_d_e))
        print('dim alpha_e: ',np.shape(alpha_e))
        print('dim alpha1: ',np.shape(np.dot(Rot_e_bigdim.T,alpha_e)))
        print('dim alpha2: ',np.shape(np.dot(Rot_e_dot_bigdim.T,np.dot(Rot_e_bigdim,get_endpoint_velocities()))))
        
        print(h_c)
        """
        robot.set_joint_torques(dict(list(zip(robot.joint_names(),total_torque))))

        rate.sleep()

        if _iterations % 10 == 0:
            print(_iterations,':    deviation fom desired position = ',p_delta)
            print('')

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




if __name__ == "__main__":

    rospy.init_node("ts_control_sim_only")

    #rospy.on_shutdown(_on_shutdown)
    publish_rate = 100
    rate = rospy.Rate(publish_rate)

    robot = ArmInterface()
    print('Moving to neutral position')
    robot.move_to_neutral()

    impedance_control(rate,K_Pt,K_Po,K_m,K_d)