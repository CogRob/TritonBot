# This file should be copied to /etc/profile.d

if [ -d /opt/cogrob/shell/profile.d ]; then
  for i in /opt/cogrob/shell/profile.d/*.sh; do
    if [ -r $i ]; then
      . $i
    fi
  done
  unset i
fi
