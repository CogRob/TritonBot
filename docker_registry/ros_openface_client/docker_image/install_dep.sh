#!/usr/bin/env bash
set -e

export DEBIAN_FRONTEND=noninteractive
apt-get update

apt-get -y install ros-indigo-grpc ros-indigo-cv-bridge

pip install grpcio absl-py

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*
