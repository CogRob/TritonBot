#!/usr/bin/env bash
set -e

export DEBIAN_FRONTEND=noninteractive
apt-get update

apt-get install --yes ros-indigo-desktop-full
pip install catkin-tools

apt-get -y install --yes ros-indigo-usb-cam ros-indigo-carrot-planner ros-indigo-urg-node
pip install --ignore-installed numpy six
pip install --upgrade scipy sklearn grpcio grpcio-tools absl-py

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*
