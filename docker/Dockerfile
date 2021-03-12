# FROM osrf/ros:kinetic-desktop-full-xenial
FROM osrf/ros:melodic-desktop-full-bionic

ENV ROS_DISTRO=${ROS_DISTRO}

RUN apt-get update && apt-get install -q -y \
    build-essential git swig sudo python-future libcppunit-dev python-pip

RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-libfranka ros-$ROS_DISTRO-franka-ros \
    python-catkin-tools ros-$ROS_DISTRO-gazebo-ros-control \
    ros-${ROS_DISTRO}-rospy-message-converter ros-${ROS_DISTRO}-effort-controllers \
    ros-${ROS_DISTRO}-joint-state-controller python-pip \
    ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-moveit-commander \
    ros-${ROS_DISTRO}-moveit-visual-tools

RUN pip install --upgrade pip

RUN pip install --upgrade numpy numpy-quaternion==2020.5.11.13.33.35

RUN apt-get update && apt-get upgrade -y

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

LABEL PS_VERSION v1.0.0-melodic

# setup entrypoint, need entrypoint.sh in the same folder with Dockerfile
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]