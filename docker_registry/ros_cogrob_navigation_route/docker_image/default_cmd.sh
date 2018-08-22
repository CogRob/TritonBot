#!/usr/bin/env bash
set -e

export TZ=America/Los_Angeles

roslaunch cogrob_navigation_route move_to_nav_point.launch --wait
