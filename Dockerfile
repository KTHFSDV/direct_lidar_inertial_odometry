#!/usr/bin/env docker

# Dockerfile for building a ROS 2 Jazzy image for your package.
# Main purpose: install dependencies and prepare a colcon workspace.

FROM ros:jazzy-ros-base

ENV ROS_ROOT=/opt/ros/jazzy
ENV ROS_PACKAGE_NAME=${PACKAGE_NAME}

# Create workspace
RUN mkdir -p /ws/src
WORKDIR /ws

# Set noninteractive installation
ENV DEBIAN_FRONTEND=noninteractive

# Install apt dependencies
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    git \
    nano \
    cmake \
    libeigen3-dev \
    libomp-dev \
    libpcl-dev \
    ros-jazzy-pcl-ros \
    ros-jazzy-tf2-eigen \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Optional: Run a script to install extra dependencies
# COPY scripts/install.sh scripts/
# RUN chmod +x scripts/install.sh && bash scripts/install.sh

WORKDIR /ws