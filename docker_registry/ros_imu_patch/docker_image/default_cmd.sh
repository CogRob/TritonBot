#!/usr/bin/env bash
set -e

export TZ=America/Los_Angeles

roslaunch imu_patch imu_patch.launch --wait
