#!/usr/bin/env bash
set -e

GITHUB_PUBKEY="AAAAB3NzaC1yc2EAAAABIwAAAQEAq2A7hRGmdnm9tUDbO9IDSwBK6TbQa+PXYPCP\
y6rbTrTtw7PHkccKrpp0yVhp5HdEIcKr6pLlVDBfOLX9QUsyCOV0wzfjIJNlGEYsdlLJizHhbn2mUjv\
SAHQqZETYP81eFzLQNnPHt4EVVUh7VfDESU84KezmD5QlWpXLmvU31/yMf+Se8xhHTvKSCZIFImWwoG\
6mbUoWf9nzpIoaSjB+weqqUUmpaaasXVal72J+UX2B+2RPW3RcT0eOzQgqlJL3RKrTJvdsjE3JEAvGq\
3lGHSZXy28G3skua2SmVi/w4yCE6gbODqnTWlg7+wC604ydGXA8VJiS5ap43JXiUFFAaQ=="

if [[ -z $(cat ~/.ssh/known_hosts 2>/dev/null | grep $GITHUB_PUBKEY) ]]; then
  mkdir -p ~/.ssh
  GITHUB_PUBKEY_STR=`ssh-keyscan -H github.com`
  if [[ -n $(echo $GITHUB_PUBKEY_STR | grep $GITHUB_PUBKEY) ]]; then
    echo $GITHUB_PUBKEY_STR >> ~/.ssh/known_hosts 2>/dev/null
  else
    echo "GitHub pubkey mismatch."
    exit 1
  fi
fi

if [ -f $HOME/.ssh/id_rsa ]; then
  chmod 600 $HOME/.ssh/id_rsa
fi
