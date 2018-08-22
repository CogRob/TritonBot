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

FROM ros:indigo-robot-trusty

MAINTAINER Shengye Wang

# Repare basic environment for running anything in Docker.
COPY run_env_prep.sh /root/install/
RUN /root/install/run_env_prep.sh
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8
ENV TZ America/Los_Angeles

# Install Mate to provide a desktop environment.
COPY install_mate.sh /root/install/
RUN /root/install/install_mate.sh

# Install TurboVNC and VirtualGL to provide VNC environment.
COPY install_turbovnc_virtualgl.sh /root/install/
RUN /root/install/install_turbovnc_virtualgl.sh
ENV PATH ${PATH}:/opt/VirtualGL/bin:/opt/TurboVNC/bin

# Prepares GitHub.
COPY github_prep.sh /root/install/
RUN /root/install/github_prep.sh

# Install Bazel.
COPY install_bazel.sh /root/install/
RUN /root/install/install_bazel.sh

# Prepare basic requirements for dockerlogin tools.
COPY opt_cogrob.sh /etc/profile.d/
COPY dockerlogin_prep.sh /root/install/
RUN /root/install/dockerlogin_prep.sh

# Prepare ROS-specific tools.
COPY install_ros_extra.sh /root/install/
RUN /root/install/install_ros_extra.sh
# Patch roscore not to start rosout.
COPY patch/roscore.xml /opt/ros/indigo/etc/ros/roscore.xml
COPY patch/roscore.xml /opt/ros/indigo/share/roslaunch/resources/roscore.xml

# nvidia-container-runtime, will only take effect with nvidia-docker
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility,graphics
COPY glvnd_devel_prep.sh /root/install/
RUN /root/install/glvnd_devel_prep.sh
COPY --from=nvidia/opengl:1.0-glvnd-runtime-ubuntu14.04 /usr/local/lib/x86_64-linux-gnu /usr/local/lib/glvnd/x86_64-linux-gnu
COPY --from=nvidia/opengl:1.0-glvnd-runtime-ubuntu14.04 /usr/local/lib/i386-linux-gnu /usr/local/lib/glvnd/i386-linux-gnu
COPY --from=nvidia/opengl:1.0-glvnd-devel-ubuntu14.04 /usr/local/include/GL /usr/local/include/GL
COPY --from=nvidia/opengl:1.0-glvnd-devel-ubuntu14.04 /usr/local/include/EGL /usr/local/include/EGL
COPY --from=nvidia/opengl:1.0-glvnd-devel-ubuntu14.04 /usr/local/include/KHR /usr/local/include/KHR
COPY patch/10_nvidia.json /usr/local/share/glvnd/egl_vendor.d/10_nvidia.json

RUN rm -rf /root/install
