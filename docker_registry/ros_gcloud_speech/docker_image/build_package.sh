#!/usr/bin/env bash
set -e

export DEBIAN_FRONTEND=noninteractive
apt-get update

CATKIN_GRPC_REPO=/root/catkin_grpc
git clone https://github.com/CogRob/catkin_grpc.git $CATKIN_GRPC_REPO

GCLOUD_SPEECH_REPO=/root/gcloud_speech
git clone https://github.com/CogRob/gcloud_speech.git $GCLOUD_SPEECH_REPO

CATKIN_SRC_PATH=/root/catkin_ws/src

mkdir -p $CATKIN_SRC_PATH

mv $CATKIN_GRPC_REPO/grpc $CATKIN_SRC_PATH/

mv $GCLOUD_SPEECH_REPO/gcloud_speech $CATKIN_SRC_PATH/
mv $GCLOUD_SPEECH_REPO/gcloud_speech_msgs $CATKIN_SRC_PATH/
mv $GCLOUD_SPEECH_REPO/gcloud_speech_utils $CATKIN_SRC_PATH/

rm -rf $CATKIN_GRPC_REPO
rm -rf $GCLOUD_SPEECH_REPO

cd $CATKIN_SRC_PATH/..

rosdep update
rosdep install --from-paths src --ignore-src --rosdistro indigo -y

catkin build

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*
