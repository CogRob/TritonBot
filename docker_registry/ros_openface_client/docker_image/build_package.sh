#!/usr/bin/env bash
set -e

TRITONBOT_REPO=/root/TritonBot
git clone https://github.com/CogRob/TritonBot.git $TRITONBOT_REPO
COGROB_ROS=/root/TritonBot/cogrob_ros

CATKIN_SRC_PATH=/root/catkin_ws/src

ln -s $COGROB_ROS/openface_client $CATKIN_SRC_PATH/openface_client
ln -s $COGROB_ROS/cogrob_face_msgs $CATKIN_SRC_PATH/cogrob_face_msgs

cd $CATKIN_SRC_PATH && catkin build
