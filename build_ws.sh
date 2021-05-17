#! /bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
cd ..
wstool init
wstool merge panda_simulator/dependencies.rosinstall
wstool up

# use old ros-compatible version of kdl
cd .. && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

source /opt/ros/$ROS_DISTRO/setup.bash
catkin build
