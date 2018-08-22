if [ -z "$ROS_ROOT" ]; then
  if [ -n "$BASH" ] && [ -f /opt/ros/indigo/setup.bash ]; then
    source /opt/ros/indigo/setup.bash
  elif [ -n "$ZSH_NAME" ] && [ -f /opt/ros/indigo/setup.zsh ]; then
    source /opt/ros/indigo/setup.zsh
  fi
fi
