#!/usr/bin/env bash
set -e

export TZ=America/Los_Angeles

exec /root/cogrob_bin/psutil_monitor_main
