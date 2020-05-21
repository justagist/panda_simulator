# FROM osrf/ros:kinetic-desktop-full # rviz and gazebo throws opengl errors
FROM nvidia/cudagl:10.2-devel-ubuntu16.04
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# -------------------------------- ROS base ---------------------------------------
# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys F42ED6FBAB17C654
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros-core \
    && rm -rf /var/lib/apt/lists/*


# install ros packages
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros-base \
    && rm -rf /var/lib/apt/lists/*

# -------------------------------------------------------------------------------

RUN apt-get update && apt-get install -q -y \
    build-essential git swig sudo python-future libcppunit-dev python-pip

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-libfranka ros-${ROS_DISTRO}-franka-ros \
    python-catkin-tools ros-${ROS_DISTRO}-gazebo-ros-control \
    ros-${ROS_DISTRO}-rospy-message-converter ros-${ROS_DISTRO}-effort-controllers \
    ros-${ROS_DISTRO}-joint-state-controller python-pip \
    ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-moveit-commander \ 
    ros-${ROS_DISTRO}-image-proc ros-${ROS_DISTRO}-roslint \
    ros-${ROS_DISTRO}-moveit-visual-tools

RUN pip install --upgrade pip

RUN pip install --upgrade numpy numpy-quaternion rospy-message-converter

RUN apt-get update && apt-get upgrade -y

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt-get update && \
    apt-get -y install libgl1-mesa-glx libgl1-mesa-dri mesa-utils && \
    rm -rf /var/lib/apt/lists/*

LABEL PS_VERSION=v1.0.0-${ROS_DISTRO}
# setup entrypoint, need entrypoint.sh in the same folder with Dockerfile
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]