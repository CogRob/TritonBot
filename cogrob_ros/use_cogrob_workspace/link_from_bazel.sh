#!/usr/bin/env bash

BAZEL=bazel

# Get script path.
BASH_SCRIPT="${BASH_SOURCE[0]}"
while [ -h "$BASH_SCRIPT" ]; do
  BASH_SCRIPT_PATH="$( cd -P "$( dirname "$BASH_SCRIPT" )" && pwd )"
  BASH_SCRIPT="$(readlink "$BASH_SCRIPT")"
  [[ $BASH_SCRIPT != /* ]] && BASH_SCRIPT="$BASH_SCRIPT_PATH/$BASH_SCRIPT"
done
BASH_SCRIPT_PATH="$( cd -P "$( dirname "$BASH_SCRIPT" )" && pwd )"

: ${BAZEL_WORKSPACE:=$BASH_SCRIPT_PATH/../../workspace}


DEST_WORKSPACE=$BASH_SCRIPT_PATH/src/use_cogrob_workspace/workspace
rm -rf $DEST_WORKSPACE
mkdir -p $DEST_WORKSPACE

IFS=$(echo -en "\n\b")

ALL_FILES=`\
  find $BAZEL_WORKSPACE -type f -not -path '*bazel*' -and -not -path '*.git*'`

printf "Making symlink for all files.\n"
for file in $ALL_FILES; do
  SRC_FILE="$file"
  DEST_FILE="${file/$BAZEL_WORKSPACE/$DEST_WORKSPACE}"
  DEST_BASEDIR=$(dirname "$DEST_FILE")
  mkdir -p $DEST_BASEDIR
  ln -s $SRC_FILE $DEST_FILE
done

printf "Creating __init__.py recursively.\n"
for CURRENT_DIR in `find $DEST_WORKSPACE -type d`; do
  touch $CURRENT_DIR/__init__.py
done

printf "Compiling all proto files for Python.\n"
ALL_PROTO_FILES=`\find $DEST_WORKSPACE -name '*.proto' \
    -not -path '*bazel*' -and -not -path '*.git*'`
python -m grpc_tools.protoc -I$DEST_WORKSPACE \
    --python_out=$DEST_WORKSPACE --grpc_python_out=$DEST_WORKSPACE \
    $ALL_PROTO_FILES
