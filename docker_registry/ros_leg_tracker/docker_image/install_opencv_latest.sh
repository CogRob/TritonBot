#!/usr/bin/env bash
set -e

export DEBIAN_FRONTEND=noninteractive
apt-get update

apt-get install --yes unzip

mkdir -p /root/install/ocv-tmp && cd /root/install/ocv-tmp && \
  curl -L https://github.com/opencv/opencv/archive/3.4.1.zip -o ocv.zip && \
  unzip ocv.zip && \
  cd opencv-3.4.1 && \
    mkdir release && cd release && \
      cmake -D CMAKE_BUILD_TYPE=RELEASE \
            -D CMAKE_INSTALL_PREFIX=/usr/local \
            -D BUILD_PYTHON_SUPPORT=ON \
            -D CPU_AVX512_SKX_SUPPORTED=OFF \
            .. && \
      make -j8 && \
      make install && \
rm -rf /root/ocv-tmp

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*
