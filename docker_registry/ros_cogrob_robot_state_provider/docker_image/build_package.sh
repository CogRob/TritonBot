#!/usr/bin/env bash
set -e

export DEBIAN_FRONTEND=noninteractive
apt-get update

CATKIN_SRC_PATH=/root/catkin_ws/src
mkdir -p $CATKIN_SRC_PATH

TRITONBOT_REPO=/root/TritonBot
git clone https://github.com/CogRob/TritonBot.git $TRITONBOT_REPO
COGROB_ROS=/root/TritonBot/cogrob_ros

ln -s $COGROB_ROS/cogrob_robot_state_provider $CATKIN_SRC_PATH/
ln -s $COGROB_ROS/cogrob_robot_state_msgs $CATKIN_SRC_PATH/

cd $CATKIN_SRC_PATH/..

rosdep update
rosdep install --from-paths src --ignore-src --rosdistro indigo -y

catkin build

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*
