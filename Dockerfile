FROM ros:kinetic-ros-base-xenial
MAINTAINER Yusu Pan <xxdsox@gmail.com>
LABEL Description="Dockerfile for humanoid project of ZJUDancer."

# set environment variables
ENV ZJUDANCER_WS=/root/ZJUDancer/
ENV ZJUDANCER_ROBOTID=5
ENV ZJUDANCER_GUI=1
ENV ZJUDANCER_GPU=0
ENV ZJUDANCER_SIMULATION=true
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8

# install ros packages
RUN apt-get update -y
RUN apt-get install -y build-essential git wget \
        cmake libpython-dev libavcodec-dev \
        libswscale-dev libx264-dev libprotobuf-dev \
        protobuf-compiler python3-pip python-pip \
        python-dev pypy libzmq3-dev \
        libzmqpp-dev libgflags-dev
RUN apt-get install -y ros-kinetic-desktop-full python-rosinstall \
        python-rosinstall-generator python-wstool ros-kinetic-image-transport \
        ros-kinetic-tf2 ros-kinetic-tf2-geometry-msgs ros-kinetic-opencv3

# Replacing shell with bash for later docker build commands
# FIXME: find an elegant way
# RUN mv /bin/sh /bin/sh.bak && \
#     ln -s /bin/bash /bin/sh

# copy source code
RUN rm -rf $ZJUDANCER_WS/humanoid/
RUN rm -rf $ZJUDANCER_WS/humanoid-lib/
RUN mkdir -p $ZJUDANCER_WS/humanoid/
RUN mkdir -p $ZJUDANCER_WS/humanoid-lib/
COPY ZJUDancer/humanoid $ZJUDANCER_WS/humanoid/
COPY ZJUDancer/humanoid-lib $ZJUDANCER_WS/humanoid-lib/
RUN pwd
RUN ls -ll

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && \
                  ls $ZJUDANCER_WS && \
                  cd $ZJUDANCER_WS/humanoid-lib && \
                  pwd && \
                  ls -ll && \
                  catkin_make && \
                  source devel/setup.bash && \
                  cd $ZJUDANCER_WS/humanoid && \
                  pwd && \
                  ls -ll && \
                  catkin_make && \
                  source devel/setup.bash && \
                  echo '/opt/ros/kinetic/setup.bash' >> ~/.bashrc"

