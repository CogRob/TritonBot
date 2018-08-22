#!/usr/bin/env bash

if [ "$#" -ne 1 ]; then
  echo "build_and_push.sh <image_name>"
  exit 1
fi

# Get script path.
# http://stackoverflow.com/questions/59895/can-a-bash-script-tell-which-directory-it-is-stored-in
BASH_SCRIPT="${BASH_SOURCE[0]}"
while [ -h "$BASH_SCRIPT" ]; do # resolve $BASH_SCRIPT until the file is no longer a symlink
  BASH_SCRIPT_PATH="$( cd -P "$( dirname "$BASH_SCRIPT" )" && pwd )"
  BASH_SCRIPT="$(readlink "$BASH_SCRIPT")"
  # if $BASH_SCRIPT was a relative symlink, we need to resolve it relative to the path where the symlink file was located
  [[ $BASH_SCRIPT != /* ]] && BASH_SCRIPT="$BASH_SCRIPT_PATH/$BASH_SCRIPT"
done
BASH_SCRIPT_PATH="$( cd -P "$( dirname "$BASH_SCRIPT" )" && pwd )"

# Tolerate trailing "/".
IMAGE_NAME=$1
if [ "${IMAGE_NAME: -1}" == "/" ]; then
  IMAGE_NAME=${IMAGE_NAME%?}
fi

IMAGE_SRC_ROOT=$BASH_SCRIPT_PATH/$IMAGE_NAME/docker_image
LOCAL_TAG=$IMAGE_NAME
REMOTE_TAG=tritonbot.github.io/$IMAGE_NAME

if [ ! -e $IMAGE_SRC_ROOT/Dockerfile ]; then
  printf "$IMAGE_SRC_ROOT/Dockerfile does not exist.\n"
  exit 2
fi

EXTRA_BUILD_FLAGS=""
if [ ! -z "$COGROB_ROS_BRANCH" ]; then
  EXTRA_BUILD_FLAGS="$EXTRA_BUILD_FLAGS --build-arg COGROB_ROS_BRANCH=$COGROB_ROS_BRANCH"
fi

docker build --tag $LOCAL_TAG --tag $REMOTE_TAG $EXTRA_BUILD_FLAGS $IMAGE_SRC_ROOT
if [ $? -ne 0 ]; then
  printf "'docker build' failed.\n"
  exit $?
fi

exit $?
