#! /bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
cd ..
wstool init
wstool merge panda_simulator/dependencies.rosinstall
wstool up

# use old ros-compatible version of kdl
cd orocos_kinematics_dynamics && rm -rf * && git checkout b35c424e && git reset --hard
cd ../.. && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys python-sip

source /opt/ros/$ROS_DISTRO/setup.bash
catkin build
