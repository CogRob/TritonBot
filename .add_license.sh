#!/usr/bin/env bash

# Get script path.
BASH_SCRIPT="${BASH_SOURCE[0]}"
while [ -h "$BASH_SCRIPT" ]; do
  BASH_SCRIPT_PATH="$( cd -P "$( dirname "$BASH_SCRIPT" )" && pwd )"
  BASH_SCRIPT="$(readlink "$BASH_SCRIPT")"
  [[ $BASH_SCRIPT != /* ]] && BASH_SCRIPT="$BASH_SCRIPT_PATH/$BASH_SCRIPT"
done
BASH_SCRIPT_PATH="$( cd -P "$( dirname "$BASH_SCRIPT" )" && pwd )"

for FILE in `find $BASH_SCRIPT_PATH -type f -not -path "$BASH_SCRIPT_PATH/.git/*"`; do
  if ! grep -e 'Copyright .* The Regents of the University of California' $FILE; then
    if [[ $FILE == *.cc ]] || [[ $FILE == *.h ]] || [[ $FILE == *.proto ]]; then
      cat $BASH_SCRIPT_PATH/.LICENSE.slash $FILE > /tmp/tmp_license
      cat /tmp/tmp_license > $FILE
    elif  [[ $FILE == */CMakeLists.txt ]] || [[ $FILE == */Dockerfile ]]; then
      cat $BASH_SCRIPT_PATH/.LICENSE.pound $FILE > /tmp/tmp_license
      cat /tmp/tmp_license > $FILE
    elif [[ $FILE == *.py ]]; then
      if head -n 1 $FILE | grep "^#!"; then
        head -n 1 $FILE > /tmp/tmp_license
        cat $BASH_SCRIPT_PATH/.LICENSE.pound >> /tmp/tmp_license
        tail -n +2 "$FILE" >> /tmp/tmp_license
        cat /tmp/tmp_license > $FILE
      else
        cat $BASH_SCRIPT_PATH/.LICENSE.pound $FILE > /tmp/tmp_license
        cat /tmp/tmp_license > $FILE
      fi
    fi
  fi
done
