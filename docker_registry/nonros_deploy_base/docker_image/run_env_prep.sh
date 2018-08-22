#!/usr/bin/env bash
set -e

export DEBIAN_FRONTEND=noninteractive
apt-get update

# Install essential software.
apt-get -y install vim curl wget git openssh-client tmux screen

# Install essential library.
apt-get -y install libunwind-dev

# Install python dependencies.
apt-get -y install python-pip
pip install --upgrade pip

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*
