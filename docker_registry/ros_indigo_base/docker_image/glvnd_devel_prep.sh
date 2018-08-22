#!/usr/bin/env bash

set -e

export DEBIAN_FRONTEND=noninteractive
dpkg --add-architecture i386
apt-get update
apt-get install --yes --no-install-recommends \
  libxau6 libxau6:i386 \
  libxdmcp6 libxdmcp6:i386 \
  libxcb1 libxcb1:i386 \
  libxext6 libxext6:i386 \
  libx11-6 libx11-6:i386

rm -rf /var/lib/apt/lists/*
