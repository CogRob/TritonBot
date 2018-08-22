#!/usr/bin/env bash

BAZEL=bazel
SRC_WORKSPACE=$HOME/CogRob/workspace

# Get script path.
BASH_SCRIPT="${BASH_SOURCE[0]}"
while [ -h "$BASH_SCRIPT" ]; do
  BASH_SCRIPT_PATH="$( cd -P "$( dirname "$BASH_SCRIPT" )" && pwd )"
  BASH_SCRIPT="$(readlink "$BASH_SCRIPT")"
  [[ $BASH_SCRIPT != /* ]] && BASH_SCRIPT="$BASH_SCRIPT_PATH/$BASH_SCRIPT"
done
BASH_SCRIPT_PATH="$( cd -P "$( dirname "$BASH_SCRIPT" )" && pwd )"


DEST_WORKSPACE=$BASH_SCRIPT_PATH/workspace
rm -rf $DEST_WORKSPACE
mkdir -p $DEST_WORKSPACE

cd $SRC_WORKSPACE
ALL_MAYBE_FILES=$( \
  bazel query "deps(//cogrob/navigation/traffic_control:traffic_control_server_main) union \
               deps(//cogrob/monitor/psutil_monitor:psutil_monitor_main) union \
               deps(//cogrob/perception/face_db:facedb_server) union \
               deps(//cogrob/perception/openface:openface_server) union \
               deps(//cogrob/monitor/docker_monitor:docker_monitor_main) union \
               deps(//cogrob/identity/human_db:humandb_server) union
               deps(//cogrob/dialogue/intent_extractors:all) union \
               deps(//cogrob/dialogue/intent_selector:all) union \
               deps(//cogrob/dialogue/listen_for_intent:all) union \
               deps(//cogrob/dialogue/speech_recognition:all) union \
               deps(//cogrob/executive/proto:all) union \
               deps(//cogrob/identity/human_db/proto:all) union \
               deps(//cogrob/navigation/tour:all) union \
               deps(//cogrob/perception/face_db/proto:all) union \
               deps(//cogrob/universal_logger:all) union \
               deps(//cogrob/universal_logger/proto:all)" | \
  egrep "^//" | \
  egrep -v "^//external" | \
  sed -e "s/:/\//g"
)

WORKSPACE_STATUS_OUTPUT=$DEST_WORKSPACE/.WORKSPACE_GIT_STATUS
date >> $WORKSPACE_STATUS_OUTPUT
echo "CogRob/workspace branch: "$(git branch | grep \* | cut -d ' ' -f2-) \
    >> $WORKSPACE_STATUS_OUTPUT
echo $(git rev-parse HEAD)  >> $WORKSPACE_STATUS_OUTPUT
if GIT_CLEAN=$(git status --porcelain) && [ -z "$GIT_CLEAN" ]; then
  echo "Working directory clean." >> $WORKSPACE_STATUS_OUTPUT
else
  echo "Warning: Uncommitted changes in source workspace."
  echo "Uncommitted changes." >> $WORKSPACE_STATUS_OUTPUT
fi

for file in $ALL_MAYBE_FILES; do
  SRC_FILE="${file/\/\//$SRC_WORKSPACE/}"
  DEST_FILE="${file/\/\//$DEST_WORKSPACE/}"
  SRC_BASEDIR=$(dirname "$SRC_FILE")
  DEST_BASEDIR=$(dirname "$DEST_FILE")
  if [ -f $SRC_FILE ]; then
    mkdir -p $DEST_BASEDIR
    cp $SRC_FILE $DEST_FILE
    if [ -f $SRC_BASEDIR/BUILD ]; then
      cp $SRC_BASEDIR/BUILD $DEST_BASEDIR/BUILD
    fi
  fi
done

cp $SRC_WORKSPACE/WORKSPACE $DEST_WORKSPACE/WORKSPACE
mkdir -p $DEST_WORKSPACE/third_party
cp $SRC_WORKSPACE/third_party/requirements.txt $DEST_WORKSPACE/third_party/
cp -r $SRC_WORKSPACE/third_party/absl $DEST_WORKSPACE/third_party/
cp -r $SRC_WORKSPACE/.gitignore $DEST_WORKSPACE/
