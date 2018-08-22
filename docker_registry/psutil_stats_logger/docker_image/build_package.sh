#!/usr/bin/env bash
set -e

export DEBIAN_FRONTEND=noninteractive
apt-get update

# Install essential library.
apt-get install -y libunwind-dev

TRITONBOT_REPO=/root/TritonBot
git clone https://github.com/CogRob/TritonBot.git $TRITONBOT_REPO
COGROB_WORKSPACE=/root/TritonBot/workspace

cd $COGROB_WORKSPACE/cogrob/monitor/psutil_monitor
bazel build :psutil_monitor_main
mkdir -p /root/cogrob_bin
cp -R -L $COGROB_WORKSPACE/bazel-bin/cogrob/monitor/psutil_monitor/* \
    /root/cogrob_bin/

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*
