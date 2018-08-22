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

# Upgrade CMake to 3.x.
apt-get -y install software-properties-common
add-apt-repository -y ppa:george-edison55/cmake-3.x
apt-get update
apt-get -y upgrade cmake

# Upgrade gcc/g++ to 5.x.
add-apt-repository -y ppa:ubuntu-toolchain-r/test
apt-get update
apt-get -y install gcc-5 g++-5
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 100
update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-5 100

# Install Docker
apt-get -y install apt-transport-https ca-certificates curl software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | apt-key add -
apt-key fingerprint 0EBFCD88
add-apt-repository -y "deb [arch=amd64] https://download.docker.com/linux/ubuntu trusty stable"
apt-get update
apt-get -y install docker-ce
curl -L https://github.com/docker/compose/releases/download/1.16.1/docker-compose-Linux-x86_64 -o /usr/local/bin/docker-compose
chmod +x /usr/local/bin/docker-compose

# Make a mark so that scripts can easily know whether in container.
touch /.IN_DOCKERLOGIN

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*
