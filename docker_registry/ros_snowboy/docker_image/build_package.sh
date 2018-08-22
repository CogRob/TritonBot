#!/usr/bin/env bash
set -e

export DEBIAN_FRONTEND=noninteractive
apt-get update

CATKIN_SRC_PATH=/root/catkin_ws/src
mkdir -p $CATKIN_SRC_PATH

GCLOUD_SPEECH_REPO=/root/gcloud_speech
git clone https://github.com/CogRob/gcloud_speech.git $GCLOUD_SPEECH_REPO
mv $GCLOUD_SPEECH_REPO/gcloud_speech_msgs $CATKIN_SRC_PATH/
rm -rf $GCLOUD_SPEECH_REPO

TRITONBOT_REPO=/root/TritonBot
git clone https://github.com/CogRob/TritonBot.git $TRITONBOT_REPO
mv $TRITONBOT_REPO/cogrob_ros/ros_snowboy/snowboy $CATKIN_SRC_PATH/
rm -rf $TRITONBOT_REPO

cd $CATKIN_SRC_PATH/..

rosdep update
rosdep install --from-paths src --ignore-src --rosdistro indigo -y

catkin build

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*
