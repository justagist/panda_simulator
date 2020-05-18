#! /bin/bash
ROOT_DIR="$(cd $( dirname ${BASH_SOURCE[0]} ) && pwd)"

command_exists () {
    type "$1" &> /dev/null ;
}

distro_type="kinetic"

if command_exists nvidia-docker; then
      extra_params="--runtime nvidia"
      xdocker="nvidia-docker"
      echo -e "\t[INFO] nvidia-docker exists"
else
      xdocker="docker"
      extra_params=""
      echo -e "\t[INFO] nvidia-docker does not exist (falling back to docker). Rviz and Gazebo most likely will not work!"
fi

IMAGE_NAME="ps:${distro_type}"

echo -e "\n\t[STATUS] Loading image: ${IMAGE_NAME} ..."

if ! [ -d "$HOME/.panda_sim_${distro_type}_ws/src" ]; then
    mkdir -p $HOME/.panda_sim_${distro_type}_ws/src
fi
# XAUTH=/tmp/.docker.xauth
# if [ ! -f $XAUTH ]
# then
#     xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
#     if [ ! -z "$xauth_list" ]
#     then
#         echo $xauth_list | xauth -f $XAUTH nmerge -
#     else
#         sudo touch $XAUTH
#     fi
#     sudo chmod a+r $XAUTH
# fi
$xdocker run -it \
       --user=$(id -u) \
       --env="DISPLAY" \
       --network="host" \
       --privileged \
       --env="QT_X11_NO_MITSHM=1" \
       --volume="/home/$USER:/home/$USER" \
       --volume="/etc/group:/etc/group:ro" \
       --volume="/etc/passwd:/etc/passwd:ro" \
       --volume="/etc/shadow:/etc/shadow:ro" \
       --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
       --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
       --volume="$HOME/.panda_sim_${distro_type}_ws:/home/$USER/panda_sim_ws" \
       --volume="${ROOT_DIR}:/home/$USER/panda_sim_ws/src/panda_simulator" ${extra_params} \
       --workdir="/home/$USER/panda_sim_ws/" \
       $IMAGE_NAME \
       bash 

echo -e "\nBye the byee!\n"
