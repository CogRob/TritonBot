# Copyright (c) 2018, The Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
# * Neither the name of the University of California nor the
#   names of its contributors may be used to endorse or promote products
#   derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS OF THE UNIVERSITY OF CALIFORNIA
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

FROM tritonbot.github.io/fetch_deploy_base

################################################################################

ADD https://api.github.com/repos/googlecartographer/cartographer_ros/git/refs/heads/master \
    /root/tmp/cartographer_ros_version.json

ARG CARTOGRAPHER_VERSION=master

RUN mkdir -p cartographer_ros/
RUN mkdir -p cartographer_ros/scripts/
RUN mkdir -p catkin_ws/src/cartographer_ros/cartographer_ros/
RUN mkdir -p catkin_ws/src/cartographer_ros/cartographer_ros_msgs/
RUN mkdir -p catkin_ws/src/cartographer_ros/cartographer_rviz/

RUN git clone https://github.com/googlecartographer/cartographer_ros.git /root/tmp/cartographer_ros_repo

# First, we invalidate the entire cache if googlecartographer/cartographer has
# changed. This file's content changes whenever master changes. See:
# http://stackoverflow.com/questions/36996046/how-to-prevent-dockerfile-caching-git-clone
ADD https://api.github.com/repos/googlecartographer/cartographer/git/refs/heads/master \
    cartographer_ros/cartographer_version.json

# wstool needs the updated rosinstall file to clone the correct repos.
RUN cp /root/tmp/cartographer_ros_repo/cartographer_ros.rosinstall cartographer_ros/
RUN cp /root/tmp/cartographer_ros_repo/scripts/prepare_catkin_workspace.sh cartographer_ros/scripts/
RUN CARTOGRAPHER_VERSION=$CARTOGRAPHER_VERSION \
    cartographer_ros/scripts/prepare_catkin_workspace.sh

# rosdep needs the updated package.xml files to install the correct debs.
RUN cp /root/tmp/cartographer_ros_repo/cartographer_ros/package.xml catkin_ws/src/cartographer_ros/cartographer_ros/
RUN cp /root/tmp/cartographer_ros_repo/cartographer_ros_msgs/package.xml catkin_ws/src/cartographer_ros/cartographer_ros_msgs/
RUN cp /root/tmp/cartographer_ros_repo/cartographer_rviz/package.xml catkin_ws/src/cartographer_ros/cartographer_rviz/
RUN cp /root/tmp/cartographer_ros_repo/scripts/install_debs.sh cartographer_ros/scripts/
RUN cartographer_ros/scripts/install_debs.sh && rm -rf /var/lib/apt/lists/*

# Install proto3.
RUN /catkin_ws/src/cartographer/scripts/install_proto3.sh

# Build, install, and test all packages individually to allow caching. The
# ordering of these steps must match the topological package ordering as
# determined by Catkin.
RUN cp /root/tmp/cartographer_ros_repo/scripts/install.sh cartographer_ros/scripts/
RUN cp /root/tmp/cartographer_ros_repo/scripts/catkin_test_results.sh cartographer_ros/scripts/

RUN cp -r /root/tmp/cartographer_ros_repo/cartographer_ros_msgs/. catkin_ws/src/cartographer_ros/cartographer_ros_msgs
RUN cartographer_ros/scripts/install.sh --pkg cartographer_ros_msgs && \
    cartographer_ros/scripts/install.sh --pkg cartographer_ros_msgs \
        --catkin-make-args run_tests && \
    cartographer_ros/scripts/catkin_test_results.sh build_isolated/cartographer_ros_msgs

RUN cartographer_ros/scripts/install.sh --pkg ceres-solver

RUN cartographer_ros/scripts/install.sh --pkg cartographer && \
    cartographer_ros/scripts/install.sh --pkg cartographer --make-args test

RUN cp -r /root/tmp/cartographer_ros_repo/cartographer_ros/. catkin_ws/src/cartographer_ros/cartographer_ros
RUN cartographer_ros/scripts/install.sh --pkg cartographer_ros && \
    cartographer_ros/scripts/install.sh --pkg cartographer_ros \
        --catkin-make-args run_tests && \
    cartographer_ros/scripts/catkin_test_results.sh build_isolated/cartographer_ros

RUN cp -r /root/tmp/cartographer_ros_repo/cartographer_rviz/. catkin_ws/src/cartographer_ros/cartographer_rviz
RUN cartographer_ros/scripts/install.sh --pkg cartographer_rviz && \
    cartographer_ros/scripts/install.sh --pkg cartographer_rviz \
        --catkin-make-args run_tests && \
    cartographer_ros/scripts/catkin_test_results.sh build_isolated/cartographer_rviz

# Skip installing ros_entrypoint

# A BTRFS bug may prevent us from cleaning up these directories.
# https://btrfs.wiki.kernel.org/index.php/Problem_FAQ#I_cannot_delete_an_empty_directory
RUN rm -rf cartographer_ros catkin_ws || true

RUN rm -rf /root/tmp/cartographer_ros_repo || true

################################################################################

ADD https://api.github.com/repos/googlecartographer/cartographer_fetch/git/refs/heads/master \
    /root/tmp/cartographer_fetch_version.json

RUN git clone https://github.com/googlecartographer/cartographer_fetch.git cartographer_fetch

RUN cartographer_fetch/scripts/prepare_catkin_workspace.sh
RUN cartographer_fetch/scripts/install_debs.sh && rm -rf /var/lib/apt/lists/*
RUN cartographer_fetch/scripts/install.sh --pkg cartographer_fetch && \
    cartographer_fetch/scripts/install.sh --pkg cartographer_fetch \
        --catkin-make-args run_tests && \
    cartographer_fetch/scripts/catkin_test_results.sh build_isolated/cartographer_fetch

# A BTRFS bug may prevent us from cleaning up these directories.
# https://btrfs.wiki.kernel.org/index.php/Problem_FAQ#I_cannot_delete_an_empty_directory
RUN rm -rf cartographer_fetch catkin_ws || true

################################################################################

RUN rm -rf /root/tmp || true
