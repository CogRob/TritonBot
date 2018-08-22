#!/usr/bin/env bash
set -e

export DEBIAN_FRONTEND=noninteractive
apt-get update

apt-get -y install ros-indigo-usb-cam
apt-get -y install ros-indigo-carrot-planner
apt-get -y install ros-indigo-urg-node

pip install --ignore-installed numpy six
pip install --upgrade scipy sklearn grpcio grpcio-tools absl-py

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*
