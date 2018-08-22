#!/usr/bin/env bash
set -e

source /opt/ros/indigo/setup.bash

CATKIN_WS=/root/catkin_ws

# Prepare a Catkin workspace.
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS && catkin build
