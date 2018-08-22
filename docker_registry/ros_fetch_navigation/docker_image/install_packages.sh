#!/usr/bin/env bash
set -e

export DEBIAN_FRONTEND=noninteractive
apt-get update

apt-get --yes install ros-indigo-carrot-planner ros-indigo-global-planner ros-indigo-teb-local-planner ros-indigo-eband-local-planner

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*
