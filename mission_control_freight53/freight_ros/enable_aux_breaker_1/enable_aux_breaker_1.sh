#!/bin/bash

while true; do
  if rosservice call /aux_breaker_1 "enable: true"; then
    while true; do
      echo "Just sitting here and doing nothing."
      sleep 3600
    done
  else
    echo "Wait 5 seconds and try again."
    sleep 5
  fi
done
