#!/usr/bin/env docker

# This is a template for a Dockerfile to build a docker image for your ROS package. 
# The main purpose of this file is to install dependencies for your package.

# FROM ros:noetic-ros-base-focal
# FROM dustynv/ros:noetic-ros-base-l4t-r32.4.4
# FROM ros:melodic-ros-base-bionic       ####<--- TODO: change to your base image
# FROM ros:noetic-ros-base-focal
# FROM nvidia/cuda:11.1.1-base-ubuntu20.04
# FROM nvidia/cuda:11.1-devel-ubuntu20.04
# FROM nvidia/cuda:11.1.1-base-ubuntu20.04
from dustynv/ros:noetic-ros-base-l4t-r35.4.1

# # # setup timezone
# RUN echo 'Etc/UTC' > /etc/timezone && \
#     ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
#     apt-get update && \
#     apt-get install -q -y --no-install-recommends tzdata && \
#     rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# # setup keys
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# # setup sources.list
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'


# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO noetic

# # install ros packages
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     ros-noetic-desktop-full \
#     && rm -rf /var/lib/apt/lists/*

ENV ROS_ROOT=/opt/ros/noetic   
#ENV ROS_ROOT=/opt/ros/melodic          ###<--- TODO: change to your ROS version to mach base image

# Set upp workspace variables
ENV ROS_PACKAGE_NAME=${PACKAGE_NAME}

# Set upp workspace
RUN mkdir -p /ws/src   
WORKDIR /ws

# Set noninteractive installation
ENV DEBIAN_FRONTEND=noninteractive

# Package apt dependencies
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    git \
    nano \
	mesa-utils \ 
	iputils-ping \ 
	apt-transport-https ca-certificates \
	openssh-server python3-pip exuberant-ctags \
	git vim tmux nano htop sudo curl wget gnupg2 \
	bash-completion \
    cmake \
    pciutils \
    curl \
    python3-pip \
    libeigen3-dev \
    libomp-dev \
    libpcl-dev \
    python3-catkin-tools \
    python3-osrf-pycommon \
    ros-noetic-pcl-ros \
    libomp-dev \
    ros-noetic-tf2-eigen \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# ENV CUDA_VERSION=11.1

# RUN wget https://developer.download.nvidia.com/compute/cuda/11.1.0/local_installers/cuda_11.1.0_455.23.05_linux.run
# RUN sh cuda_11.1.0_455.23.05_linux.run

# RUN    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.1-1_all.deb
# RUN    dpkg -i cuda-keyring_1.1-1_all.deb
# RUN    apt-get update
# RUN    apt-get -y install cuda-toolkit-11-1

# RUN curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg 
# RUN curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
#     sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \ 
#     tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
  
# RUN apt-get update
  
# RUN apt-get install -y nvidia-container-toolkit

ENV NVIDIA_DISABLE_REQUIRE=true

# ENV NVIDIA_REQUIRE_CUDA "cuda>=11.1 brand=tesla,driver>=418,driver<419 brand=tesla,driver>=440,driver<441 driver>=450,driver<451"
# Installing of pip dependencies
# Installing of pip dependencies
# RUN pip3 install --upgrade \
#     && pip3 install \
#     numpy \
#     catkin_tools \
#     rospkg 

#RUN pip3 install \
#     # EXAMPLE: \
#     # torch \
#     # torchvision \
#     # tensorboardX \
#     # opencv-python \
#     # scikit-image \
#     # scikit-learn \

# Setting environment variables for PyCuda Installation
ENV PATH="/usr/local/cuda/in:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH}"


# Optional: Install additional dependencies with script
#COPY scripts/install.sh scripts/
#RUN chmod +x scripts/install.sh && bash .scripts/install.sh

WORKDIR /ws
