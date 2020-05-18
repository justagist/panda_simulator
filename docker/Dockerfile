FROM osrf/ros:kinetic-desktop-full
# FROM osrf/ros:melodic-desktop-full-bionic

ENV ROS_DISTRO=${ROS_DISTRO}

RUN apt-get update && apt-get install -q -y \
    build-essential git swig sudo python-future libcppunit-dev python-pip

RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-libfranka ros-$ROS_DISTRO-franka-ros \
    python-catkin-tools ros-$ROS_DISTRO-gazebo-ros-control \
    ros-${ROS_DISTRO}-rospy-message-converter ros-${ROS_DISTRO}-effort-controllers \
    ros-${ROS_DISTRO}-joint-state-controller python-pip \
    ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-moveit-commander

RUN pip install --upgrade pip

RUN pip install --upgrade numpy numpy-quaternion rospy-message-converter && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get upgrade -y

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt-get update && \
    apt-get -y install libgl1-mesa-glx libgl1-mesa-dri mesa-utils && \
    rm -rf /var/lib/apt/lists/*

# setup entrypoint, need entrypoint.sh in the same folder with Dockerfile
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]