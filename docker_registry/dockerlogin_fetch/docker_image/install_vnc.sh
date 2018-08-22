#!/usr/bin/env bash
set -e

export DEBIAN_FRONTEND=noninteractive
apt-get update

# Install essential software.
apt-get -y install lxde x11vnc xvfb

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*
