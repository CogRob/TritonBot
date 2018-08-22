#!/usr/bin/env bash
set -e

export DEBIAN_FRONTEND=noninteractive

cd /root/install

apt-get update
apt-get install -y \
  build-essential \
  cmake \
  curl \
  gfortran \
  git \
  graphicsmagick \
  libgraphicsmagick1-dev \
  libatlas-dev \
  libavcodec-dev \
  libavformat-dev \
  libboost-all-dev \
  libgtk2.0-dev \
  libjpeg-dev \
  liblapack-dev \
  libswscale-dev \
  pkg-config \
  python-dev \
  python-numpy \
  python-protobuf\
  software-properties-common \
  zip

curl -s https://raw.githubusercontent.com/torch/ezinstall/master/install-deps | bash -e

git clone https://github.com/torch/distro.git /opt/torch --recursive
cd /opt/torch &&
  ./install.sh && \
  cd install/bin && \
    ./luarocks install nn && \
    ./luarocks install dpnn && \
    ./luarocks install image && \
    ./luarocks install optim && \
    ./luarocks install csvigo && \
    ./luarocks install torchx && \
    ./luarocks install tds
ln -s /opt/torch/install/bin/* /usr/local/bin

mkdir -p /root/install/ocv-tmp && cd /root/install/ocv-tmp && \
  curl -L https://github.com/Itseez/opencv/archive/2.4.11.zip -o ocv.zip && \
  unzip ocv.zip && \
  cd opencv-2.4.11 && \
    mkdir release && cd release && \
      cmake -D CMAKE_BUILD_TYPE=RELEASE \
            -D CMAKE_INSTALL_PREFIX=/usr/local \
            -D BUILD_PYTHON_SUPPORT=ON \
            .. && \
      make -j8 && \
      make install && \
rm -rf /root/ocv-tmp

mkdir -p /root/install/dlib-tmp && cd /root/install/dlib-tmp && \
  curl -L \
    https://github.com/davisking/dlib/archive/v19.0.tar.gz \
    -o dlib.tar.bz2 && \
  tar xf dlib.tar.bz2 && \
  cd dlib-19.0/python_examples && \
    mkdir build && cd build && \
      cmake ../../tools/python && \
      cmake --build . --config Release && \
      cp dlib.so /usr/local/lib/python2.7/dist-packages && \
rm -rf /root/install/dlib-tmp

rm -rf /var/lib/apt/lists/*
