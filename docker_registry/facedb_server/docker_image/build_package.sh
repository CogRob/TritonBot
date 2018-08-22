#!/usr/bin/env bash
set -e

pip install --upgrade futures python-gflags glog grpcio numpy absl-py sklearn pandas scipy

TRITONBOT_REPO=/root/TritonBot
git clone https://github.com/CogRob/TritonBot.git $TRITONBOT_REPO
COGROB_WORKSPACE=/root/TritonBot/workspace

cd $COGROB_WORKSPACE/cogrob/perception/face_db
bazel build :facedb_server
mkdir -p /root/cogrob_bin
cp -R -L $COGROB_WORKSPACE/bazel-bin/cogrob/perception/face_db/* \
    /root/cogrob_bin/
