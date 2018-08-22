#!/usr/bin/env bash
set -e

export TZ=America/Los_Angeles

exec /root/cogrob_bin/traffic_control_server_main --nav_chart_pb=/home/cogrob_local/nav_chart.pb
