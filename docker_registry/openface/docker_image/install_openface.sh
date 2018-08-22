#!/usr/bin/env bash
set -e

export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y \
  curl \
  git \
  graphicsmagick \
  libssl-dev \
  libffi-dev \
  python-dev \
  python-pip \
  python-numpy \
  python-nose \
  python-scipy \
  python-pandas \
  python-protobuf \
  python-openssl \
  wget \
  zip

git clone https://github.com/cmusatyalab/openface.git /opt/openface

python -m pip install --upgrade --force pip

cd /opt/openface && \
  ./models/get-models.sh && \
  pip2 install -r requirements.txt && \
  python2 setup.py install && \
  pip2 install --user --ignore-installed -r demos/web/requirements.txt && \
  pip2 install -r training/requirements.txt

rm -rf /var/lib/apt/lists/*
