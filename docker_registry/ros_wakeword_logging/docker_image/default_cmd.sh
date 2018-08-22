#!/usr/bin/env bash
set -e

export TZ=America/Los_Angeles

rosrun wakeword_logging wakeword_logger_node
