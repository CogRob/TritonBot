#!/usr/bin/env bash
set -e

export DEBIAN_FRONTEND=noninteractive
apt-get update

apt-get -y install python-pyaudio
pip install grpcio grpcio-tools librosa
pip install -U numba

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*
