#!/usr/bin/env bash
set -e

export DEBIAN_FRONTEND=noninteractive
apt-get update

# Fix a warning.
apt-get --yes install apt-utils

# Install essential software.
apt-get --yes install vim curl wget git openssh-client tmux screen less rsync

# Install python dependencies.
apt-get -y install python-pip
pip install --upgrade pip

# Upgrade CMake to 3.x.
apt-get -y install software-properties-common
add-apt-repository -y ppa:george-edison55/cmake-3.x
apt-get update
apt-get -y upgrade cmake
cp -n /usr/share/cmake-2.8/Modules/* /usr/share/cmake-3.2/Modules/

# Upgrade gcc/g++ to 5.x.
add-apt-repository -y ppa:ubuntu-toolchain-r/test
apt-get update
apt-get -y install gcc-5 g++-5
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 100
update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-5 100

# Fix locale issue.
apt-get --yes install locales
locale-gen en_US en_US.UTF-8
dpkg-reconfigure locales

# Set the timezone.
apt-get install tzdata
ln -snf /usr/share/zoneinfo/America/Los_Angeles /etc/localtime
echo "America/Los_Angeles" > /etc/timezone

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*
