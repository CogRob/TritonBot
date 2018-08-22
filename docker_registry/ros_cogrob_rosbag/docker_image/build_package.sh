#!/usr/bin/env bash
set -e

CATKIN_SRC_PATH=/root/catkin_ws/src

TRITONBOT_REPO=/root/TritonBot
git clone https://github.com/CogRob/TritonBot.git $TRITONBOT_REPO
COGROB_ROS=/root/TritonBot/cogrob_ros

ln -s $COGROB_ROS/cogrob_rosbag_msgs $CATKIN_SRC_PATH/
ln -s $COGROB_ROS/cogrob_rosbag_servicer $CATKIN_SRC_PATH/

cd $CATKIN_SRC_PATH/..
catkin build
