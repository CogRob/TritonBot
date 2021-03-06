#!/usr/bin/env bash
set -e

TRITONBOT_REPO=/root/TritonBot
git clone https://github.com/CogRob/TritonBot.git $TRITONBOT_REPO
COGROB_ROS=/root/TritonBot/cogrob_ros

CATKIN_SRC_PATH=/root/catkin_ws/src

ln -s $COGROB_ROS/speak_text_msgs $CATKIN_SRC_PATH/speak_text_msgs
ln -s $COGROB_ROS/speak_text $CATKIN_SRC_PATH/speak_text

cd $CATKIN_SRC_PATH && catkin build
