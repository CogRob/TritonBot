#!/usr/bin/env bash
set -e

export DEBIAN_FRONTEND=noninteractive
apt-get update

apt-get install --yes python-scipy
pip install pykalman munkres

CATKIN_SRC_PATH=/root/catkin_ws/src
mkdir -p $CATKIN_SRC_PATH

git clone -b indigo https://github.com/angusleigh/leg_tracker.git $CATKIN_SRC_PATH/leg_tracker
sed -i 's/image_geometry/cogrob_use_system_opencv/g' $CATKIN_SRC_PATH/leg_tracker/CMakeLists.txt $CATKIN_SRC_PATH/leg_tracker/package.xml

cd $CATKIN_SRC_PATH/..

rosdep update
rosdep install --from-paths src --ignore-src --rosdistro indigo -y
apt-get install --yes ros-indigo-interactive-markers

catkin build

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*
