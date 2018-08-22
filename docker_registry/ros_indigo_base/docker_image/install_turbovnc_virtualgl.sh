#!/usr/bin/env bash

set -e

export DEBIAN_FRONTEND=noninteractive
dpkg --add-architecture i386
apt-get update

TURBOVNC_VERSION=2.1.2
VIRTUALGL_VERSION=2.5.2
LIBJPEG_VERSION=1.5.2

apt-get install --yes \
  ca-certificates curl gcc libc6-dev libglu1 libglu1:i386 libsm6 libxv1 \
  libxv1:i386 make python python-numpy x11-xkb-utils xauth xfonts-base \
  xkb-data

DEB_DIR=/tmp/turbovnc_virtualgl_install
mkdir -p $DEB_DIR
cd $DEB_DIR
curl -fsSL -O https://phoenixnap.dl.sourceforge.net/project/turbovnc/${TURBOVNC_VERSION}/turbovnc_${TURBOVNC_VERSION}_amd64.deb
curl -fsSL -O https://phoenixnap.dl.sourceforge.net/project/libjpeg-turbo/${LIBJPEG_VERSION}/libjpeg-turbo-official_${LIBJPEG_VERSION}_amd64.deb
curl -fsSL -O https://phoenixnap.dl.sourceforge.net/project/virtualgl/${VIRTUALGL_VERSION}/virtualgl_${VIRTUALGL_VERSION}_amd64.deb
curl -fsSL -O https://phoenixnap.dl.sourceforge.net/project/virtualgl/${VIRTUALGL_VERSION}/virtualgl32_${VIRTUALGL_VERSION}_amd64.deb

dpkg -i *.deb
cd $HOME
rm -rf $DEB_DIR

sed -i 's/$host:/unix:/g' /opt/TurboVNC/bin/vncserver

ln -s /usr/lib/libdlfaker.so /usr/lib/x86_64-linux-gnu/libdlfaker.so
ln -s /usr/lib/libvglfaker.so /usr/lib/x86_64-linux-gnu/libvglfaker.so

rm -rf /var/lib/apt/lists/*
