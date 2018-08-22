#!/usr/bin/env bash
set -e

export DEBIAN_FRONTEND=noninteractive
apt-get update

CATKIN_SRC_PATH=/root/catkin_ws/src
mkdir -p /root/CogRob

# gcloud_speech_msgs
GCLOUD_SPEECH_REPO=/root/CogRob/gcloud_speech
git clone https://github.com/CogRob/gcloud_speech.git $GCLOUD_SPEECH_REPO
ln -s $GCLOUD_SPEECH_REPO/gcloud_speech_msgs $CATKIN_SRC_PATH/

# gcloud_speech_logging
TRITONBOT_REPO=/root/TritonBot
git clone https://github.com/CogRob/TritonBot.git $TRITONBOT_REPO
COGROB_ROS=/root/TritonBot/cogrob_ros

ln -s $COGROB_ROS/gcloud_speech_logging $CATKIN_SRC_PATH/

rosdep update
rosdep install --from-paths $CATKIN_SRC_PATH --ignore-src -r -y
cd $CATKIN_SRC_PATH/..
catkin build

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*
