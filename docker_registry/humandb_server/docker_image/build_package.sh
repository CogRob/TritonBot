#!/usr/bin/env bash
set -e

pip install --upgrade futures python-gflags glog grpcio numpy absl-py

TRITONBOT_REPO=/root/TritonBot
git clone https://github.com/CogRob/TritonBot.git $TRITONBOT_REPO
COGROB_WORKSPACE=/root/TritonBot/workspace

cd $COGROB_WORKSPACE/cogrob/identity/human_db
bazel build :humandb_server
mkdir -p /root/cogrob_bin
cp -R -L $COGROB_WORKSPACE/bazel-bin/cogrob/identity/human_db/* \
    /root/cogrob_bin/
