#!/usr/bin/env bash
set -e

export TZ=America/Los_Angeles

rosrun gcloud_speech_logging gcloud_speech_logger_node
