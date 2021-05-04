#!/bin/sh
''':'
if [ "$ROS_PYTHON_VERSION" = "3" ]; then
  exec python3 "$0" "$@"
else
  exec python2 "$0" "$@"
fi
'''

import rospy
from franka_core_msgs.msg import JointCommand

if __name__ == "__main__":
    
    rospy.init_node("force_neutral_pose_startup")

    pub = rospy.Publisher('panda_simulator/motion_controller/arm/joint_commands',
        JointCommand,
        tcp_nodelay=True,
        queue_size=10)

    command_msg = JointCommand()
    command_msg.names = ["panda_joint%d" % (idx) for idx in range(1, 8)]
    command_msg.position = [0.000,- 0.785,0.0,- 2.356,0.0,1.57,0.785]
    command_msg.mode = JointCommand.POSITION_MODE

    
    rospy.sleep(0.5)
    start = rospy.Time.now().to_sec()

    rospy.loginfo("Attempting to force robot to neutral pose...")
    rospy.sleep(0.5)
    
    while not rospy.is_shutdown() and (rospy.Time.now().to_sec() - start < 1.):
        # print rospy.Time.now()
        command_msg.header.stamp = rospy.Time.now()
        pub.publish(command_msg)

    rospy.loginfo("Robot forced to neutral pose. Complete!")
