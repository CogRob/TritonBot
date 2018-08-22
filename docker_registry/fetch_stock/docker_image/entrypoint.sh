#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/indigo/setup.bash"

exec "$@"
