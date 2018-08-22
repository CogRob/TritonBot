#!/usr/bin/env bash
set -e

export DEBIAN_FRONTEND=noninteractive
apt-get update

# Allow any user to access "sudo" without password.
apt-get -y install sudo
echo "ALL ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/allow_all

# Install essential software.
apt-get -y install zsh vim emacs curl wget git tmux screen byobu

# Install python dependencies.
apt-get -y install python-pip
pip install --upgrade pip

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*
