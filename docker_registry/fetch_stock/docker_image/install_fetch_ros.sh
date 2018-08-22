#!/usr/bin/env bash

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get -y install wget

echo "deb http://packages.ros.org/ros/ubuntu trusty main" >> \
    /etc/apt/sources.list.d/ros-latest.list
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 \
    --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

echo "deb http://packages.fetchrobotics.com/ros/ubuntu/ trusty main" >> \
    /etc/apt/sources.list.d/ros-latest.list
wget --quiet http://packages.fetchrobotics.com/ros.key -O - | apt-key add -

echo "deb http://packages.fetchrobotics.com/fetch/ubuntu/ trusty main" >> \
    /etc/apt/sources.list.d/fetch-latest.list
wget --quiet http://packages.fetchrobotics.com/fetch.key -O - | apt-key add -

apt-get update

apt-get -y install python-dev python-pip libssl-dev libffi-dev
pip install --upgrade pip
pip install six --upgrade
pip install tornado-couchdb pyopenssl ndg-httpsclient pyasn1 pyparsing appdirs

mkdir -p /etc/acpi

apt-get -y install \
  fetch-system-config \
  ros-indigo-fetch-calibration \
  ros-indigo-fetch-moveit-config

apt-get -y install \
  ros-indigo-compressed-image-transport \
  ros-indigo-compressed-depth-image-transport \
  ros-indigo-theora-image-transport

pip install catkin-tools

# Fix locale issue.
apt-get -y install locales
locale-gen en_US en_US.UTF-8
dpkg-reconfigure locales

# Install essential software.
apt-get -y install curl wget git

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*
