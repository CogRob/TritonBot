#!/usr/bin/env bash

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get -y install wget

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

apt-get -y install ros-indigo-desktop-full

apt-get -y install \
    ros-indigo-fetch-description \
    ros-indigo-fetch-calibration \
    ros-indigo-freight-calibration \
    ros-indigo-fetch-moveit-config \
    ros-indigo-fetch-navigation \
    ros-indigo-fetch-bringup \
    ros-indigo-freight-bringup \
    ros-indigo-fetch-auto-dock

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*
