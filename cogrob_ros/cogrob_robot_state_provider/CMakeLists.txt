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

cmake_minimum_required(VERSION 2.8.3)
project(cogrob_robot_state_provider)

find_package(
  catkin REQUIRED
  COMPONENTS abseil_cpp roscpp cogrob_robot_state_msgs power_msgs tf grpc)

if(CMAKE_VERSION VERSION_LESS "3.1")
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")
  else()
    message(SEND_ERROR "Error: Can not enable C++ 11.")
  endif()
else()
  set(CMAKE_CXX_STANDARD 11)
endif()

catkin_package()

include_directories(src workspace ${catkin_INCLUDE_DIRS})

generate_proto(robot_status_protos GRPC
  SRC_BASE workspace
  workspace/cogrob/navigation/proto/navigation_chart.proto
  workspace/cogrob/navigation/proto/occupancy_grid.proto
  workspace/cogrob/robot_status/proto/battery_status.proto
  workspace/cogrob/robot_status/proto/robot_status.proto)

add_executable(cogrob_robot_state_provider_node
  src/battery_state_servicer.cc
  src/cogrob_robot_state_provider_node.cc
  src/robot_position_servicer.cc
  src/robot_status_impl.cc
)

add_library(cogrob_robot_state_provider_node_workspace_cc
  workspace/util/status.cc
  workspace/util/statusor.cc
)

target_link_libraries(cogrob_robot_state_provider_node
  robot_status_protos cogrob_robot_state_provider_node_workspace_cc
  pthread glog gflags ${catkin_LIBRARIES})
