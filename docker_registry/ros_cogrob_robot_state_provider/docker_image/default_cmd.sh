#!/usr/bin/env bash
set -e

export TZ=America/Los_Angeles

rosrun cogrob_robot_state_provider cogrob_robot_state_provider_node
