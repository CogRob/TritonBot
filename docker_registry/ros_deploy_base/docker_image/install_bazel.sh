#!/usr/bin/env bash
set -e

export DEBIAN_FRONTEND=noninteractive
apt-get update

apt-get -y install software-properties-common debconf-utils curl
add-apt-repository -y ppa:webupd8team/java
apt-get update

echo "debconf shared/accepted-oracle-license-v1-1 select true" \
    | debconf-set-selections
echo "debconf shared/accepted-oracle-license-v1-1 seen true" \
    | debconf-set-selections
apt-get install -y oracle-java8-installer

echo "deb [arch=amd64] http://storage.googleapis.com/bazel-apt stable jdk1.8" \
    | tee /etc/apt/sources.list.d/bazel.list
curl https://bazel.build/bazel-release.pub.gpg | apt-key add -

apt-get update
apt-get -y install bazel

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*
