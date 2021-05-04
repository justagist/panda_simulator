#!/bin/sh
''':'
if [ "$ROS_PYTHON_VERSION" = "3" ]; then
  exec python3 "$0" "$@"
else
  exec python2 "$0" "$@"
fi
'''
import rospy
from sensor_msgs.msg import JointState
from franka_core_msgs.msg import JointCommand, RobotState
import numpy as np
from copy import deepcopy

vals = []
vels = []
names = ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']

neutral_pose = [-0.017792060227770554, -0.7601235411041661, 0.019782607023391807, -2.342050140544315, 0.029840531355804868, 1.5411935298621688, 0.7534486589746342]

def callback(msg):

    global vals , vels
    temp_vals = []
    temp_vels = []
    for n in names:
        idx = msg.name.index(n)
        temp_vals.append(msg.position[idx])
        temp_vels.append(msg.velocity[idx])

    vals = deepcopy(temp_vals)
    vels = deepcopy(temp_vels)



t = 0
def state_callback(msg):
    global t
    if t%100 == 0:
        t = 1
        rospy.loginfo("============= Current robot state: ============\n" )
        rospy.loginfo("Cartesian vel: \n{}\n".format(msg.O_dP_EE) )
        rospy.loginfo("Gravity compensation torques: \n{}\n".format(msg.gravity) )
        rospy.loginfo("Coriolis: \n{}\n".format(msg.coriolis) )
        rospy.loginfo("Inertia matrix: \n{}\n".format(msg.mass_matrix) )
        rospy.loginfo("Zero Jacobian: \n{}\n".format(msg.O_Jac_EE) )


        rospy.loginfo("\n\n========\n\n")

    t+=1

def send_to_neutral():
    """
        DON'T USE THIS ON REAL ROBOT!!! 

    """
    temp_pub = rospy.Publisher('/panda_simulator/motion_controller/arm/joint_commands',JointCommand, queue_size = 1, tcp_nodelay = True)
    # Create JointCommand message to publish commands
    pubmsg = JointCommand()
    pubmsg.names = names # names of joints (has to be 7)
    pubmsg.position = neutral_pose # JointCommand msg has other fields (velocities, efforts) for
                           # when controlling in other control mode
    pubmsg.mode = pubmsg.POSITION_MODE # Specify control mode (POSITION_MODE, VELOCITY_MODE, IMPEDANCE_MODE (not available in sim), TORQUE_MODE)
    curr_val = deepcopy(vals)

    while all(abs(neutral_pose[i]-curr_val[i]) > 0.01 for i in range(len(curr_val))):
        temp_pub.publish(pubmsg)
        curr_val = deepcopy(vals)

if __name__ == '__main__':
    

    rospy.init_node("test_node")

    rospy.wait_for_service('/controller_manager/list_controllers')

    rospy.loginfo("Starting node...")
    rospy.sleep(5)

    pub = rospy.Publisher('/panda_simulator/motion_controller/arm/joint_commands',JointCommand, queue_size = 1, tcp_nodelay = True)

    # Subscribe to robot joint state
    rospy.Subscriber('/panda_simulator/custom_franka_state_controller/joint_states', JointState, callback)

    # Subscribe to robot state (Refer JointState.msg to find all available data. 
    # Note: All msg fields are not populated when using the simulated environment)
    rospy.Subscriber('/panda_simulator/custom_franka_state_controller/robot_state', RobotState, state_callback)
    

    rate = rospy.Rate(1000)

    max_val = 1.1897
    min_val = 0.4723473991867569

    rospy.loginfo("Not recieved current joint state msg")

    while not rospy.is_shutdown() and len(vals) != 7:
        continue

    rospy.loginfo("Sending robot to neutral pose")
    send_to_neutral() # DON'T DO THIS ON REAL ROBOT!! (use move_to_neutral() method from ArmInterface of franka_ros_interface package)

    rospy.sleep(2.0)

    initial_pose = vals

    rospy.loginfo("Commanding...\n")
    elapsed_time_ = rospy.Duration(0.0)
    period = rospy.Duration(0.005)

    count = 0

    # Create JointCommand message to publish commands
    pubmsg = JointCommand()
    pubmsg.names = names # names of joints (has to be 7 and in the same order as the command fields (positions, velocities, efforts))
    while not rospy.is_shutdown():

        elapsed_time_ += period

        delta = 3.14 / 16.0 * (1 - np.cos(3.14 / 5.0 * elapsed_time_.to_sec())) * 0.2

        for j in range(len(vals)):
            if j == 4:
                vals[j] = initial_pose[j] - delta
            else:
                vals[j] = initial_pose[j] + delta

        pubmsg.position = vals # JointCommand msg has other fields (velocities, efforts) for
                               # when controlling in other control mode
        # pubmsg.effort = [0.,0.,0.,0.,0.,0.,0.]
        pubmsg.mode = pubmsg.POSITION_MODE # Specify control mode (POSITION_MODE, VELOCITY_MODE, IMPEDANCE_MODE (not available in sim), TORQUE_MODE)

        pub.publish(pubmsg)
        rate.sleep()



