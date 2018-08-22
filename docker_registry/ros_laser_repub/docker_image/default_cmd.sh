#!/usr/bin/env bash
set -e

export TZ=America/Los_Angeles

roslaunch laser_repub remap_base_back_scan.launch --wait
