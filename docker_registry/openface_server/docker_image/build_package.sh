#!/usr/bin/env bash
set -e

pip install --upgrade futures python-gflags glog grpcio numpy

TRITONBOT_REPO=/root/TritonBot
git clone https://github.com/CogRob/TritonBot.git $TRITONBOT_REPO
COGROB_WORKSPACE=/root/TritonBot/workspace

cd $COGROB_WORKSPACE/cogrob/perception/openface
bazel build :openface_server
mkdir -p /root/cogrob_bin
cp -R -L $COGROB_WORKSPACE/bazel-bin/cogrob/perception/openface/* \
    /root/cogrob_bin/
