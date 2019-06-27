#######################################################################
# Author: Nikolaus Mayer (2018), mayern@cs.uni-freiburg.de
#######################################################################

FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04

## Container's mount points for the host's input/output folders
VOLUME "/input"
VOLUME "/output"

RUN apt update                             && \
    apt install -y --no-install-recommends    \
    python3                                   \
    python3-distutils                         \
    cmake                                     \
    git                                       \
    curl                                      \
    wget                                      \
    libeigen3-dev                             \
    sudo

## Switch to non-root user 
ARG uid
ARG gid
ENV uid=${uid}
ENV gid=${gid}
ENV USER=netdef
RUN groupadd -g $gid $USER                                              && \
    mkdir -p /home/$USER                                                && \
    echo "${USER}:x:${uid}:${gid}:${USER},,,:/home/${USER}:/bin/bash"      \
         >> /etc/passwd                                                 && \
    echo "${USER}:x:${uid}:"                                               \
         >> /etc/group                                                  && \
    echo "${USER} ALL=(ALL) NOPASSWD: ALL"                                 \
         > /etc/sudoers.d/${USER}                                       && \
    chmod 0440 /etc/sudoers.d/${USER}                                   && \
    chown ${uid}:${gid} -R /home/${USER}

USER ${USER}
ENV HOME=/home/${USER}

WORKDIR ${HOME}

RUN sudo curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py   && \
    sudo python3 get-pip.py                                        && \
    wget https://lmb.informatik.uni-freiburg.de/resources/binaries/tensorflow-binaries/tensorflow-1.11.0-cp36-cp36m-linux_x86_64.whl && \
    sudo -H pip3 install ${HOME}/tensorflow-1.11.0-cp36-cp36m-linux_x86_64.whl && \
    sudo -H pip3 install scikit-learn pillow scipy==1.2.0 

RUN git clone https://github.com/lmb-freiburg/lmbspecialops && \
    cd lmbspecialops                                        && \
    git checkout 3e01ebaf0da6a5d0545f1ffead4bccdbe79a26f5   && \
    find . -type f -print0 | xargs -0 sed -i 's/data.starts_with(/str_util::StartsWith(data,/g' && \
    find . -type f -print0 | xargs -0 sed -i 's/^set_target_properties.*GLIBCXX_USE_CXX11_ABI.*/#/g' && \
    mkdir build                                             && \
    cd build                                                && \
    sudo ln -s /usr/local/cuda/lib64/stubs/libcuda.so /usr/local/cuda/lib64/stubs/libcuda.so.1 && \
    export LD_LIBRARY_PATH=/usr/local/cuda/lib64/stubs/:$LD_LIBRARY_PATH && \
    cmake ..                                                && \
    make -j                                                 && \
    sudo rm /usr/local/cuda/lib64/stubs/libcuda.so.1

RUN git clone https://github.com/lmb-freiburg/netdef_slim   && \
    cd netdef_slim                                          && \
    git checkout 54f101d0f6a0bb1b815b808754176e2732e8de77   && \
    cd ..                                                   && \
    git clone https://github.com/lmb-freiburg/netdef_models && \
    cd netdef_models                                        && \
    git checkout 204add373a1a8070e082112a990cb553123b79af   && \
    cd DispNet3  && bash download_snapshots.sh && cd ..  #   && \
    # cd FlowNet3  && bash download_snapshots.sh && cd ..     && \
    # cd FlowNetH  && bash download_snapshots.sh && cd ..
    #cd SceneFlow && bash download_snapshots.sh && cd ..

# Install ROS dependencies
#  Copied from https://github.com/osrf/docker_images/blob/master/ros/melodic/ubuntu/bionic/ros-core/Dockerfile

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

# install packages
RUN sudo apt-get update && sudo apt-get install -q -y \
    dirmngr \
    gnupg2 \
    lsb-release \
    && sudo rm -rf /var/lib/apt/lists/*

# setup keys
RUN sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" | sudo tee /etc/apt/sources.list.d/ros-latest.list > /dev/null

RUN echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections

# install bootstrap tools
RUN sudo apt-get update && sudo apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    ros-melodic-ros-core=1.4.1-0* \
    ros-melodic-ros-base=1.4.1-0* \
    ros-melodic-perception=1.4.1-0* \
    && sudo rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN sudo rosdep init \
    && rosdep update

ENV ROS_DISTRO melodic

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

ENV LMBSPECIALOPS_LIB="/home/netdef/lmbspecialops/build/lib/lmbspecialops.so"
ENV PYTHONPATH="/usr/local/lib/python3.6/dist-packages:/home/netdef/lmbspecialops/python:/home/netdef"
ENV CUDA_VISIBLE_DEVICES="${GPU_IDX}"
ENV PATH="/home/netdef/netdef_slim/tools:$PATH"

RUN sudo apt-get update && sudo apt-get install python3-yaml python3-catkin-pkg-modules python3-rospkg-modules -y \
  && sudo rm -rf /var/lib/apt/lists/*

COPY src/dispnet-wrapper/dispnet_wrapper.py /home/netdef
