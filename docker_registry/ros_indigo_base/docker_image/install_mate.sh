#!/usr/bin/env bash

set -e

export DEBIAN_FRONTEND=noninteractive
apt-get update

echo "resolvconf resolvconf/linkify-resolvconf boolean false" | debconf-set-selections
apt-get install --yes resolvconf
apt-get install --yes xorg busybox xterm

apt-get install --yes software-properties-common python-software-properties
apt-add-repository --yes ppa:ubuntu-mate-dev/trusty-mate
apt-get update
apt-get install --yes ubuntu-mate-core ubuntu-mate-desktop

rm -rf /etc/xdg/autostart/blueman.desktop /etc/xdg/autostart/polkit-gnome-authentication-agent-1.desktop /etc/xdg/autostart/polkit-mate-authentication-agent-1.desktop

rm -rf /var/lib/apt/lists/*
