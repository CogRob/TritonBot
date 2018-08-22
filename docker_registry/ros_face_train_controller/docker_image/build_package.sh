#!/usr/bin/env bash
set -e

TRITONBOT_REPO=/root/TritonBot
git clone https://github.com/CogRob/TritonBot.git $TRITONBOT_REPO
COGROB_ROS=/root/TritonBot/cogrob_ros

CATKIN_SRC_PATH=/root/catkin_ws/src

ln -s $COGROB_ROS/cogrob_face_msgs $CATKIN_SRC_PATH/cogrob_face_msgs
ln -s $COGROB_ROS/cogrob_face_protos $CATKIN_SRC_PATH/cogrob_face_protos
ln -s $COGROB_ROS/face_train_controller $CATKIN_SRC_PATH/face_train_controller

cd $CATKIN_SRC_PATH && catkin build
