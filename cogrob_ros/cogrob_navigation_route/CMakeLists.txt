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
project(cogrob_navigation_route)

find_package(catkin REQUIRED COMPONENTS grpc std_msgs abseil_cpp roscpp
    abseil_cpp cogrob_navigation_msgs actionlib)

catkin_package() 

include_directories(${catkin_INCLUDE_DIRS} workspace src) 

add_library(dock_control src/dock_control.cc)
target_link_libraries(dock_control ${catkin_LIBRARIES} glog gflags)

file(GLOB_RECURSE all_proto_files workspace/*.proto)
generate_proto(
    cogrob_navigation_route_protos SRC_BASE workspace GRPC ${all_proto_files})

file(GLOB_RECURSE all_workspace_cc_files workspace/*.cc)
add_library(cogrob_navigation_route_workspace_cc_lib ${all_workspace_cc_files})

add_library(move_control src/move_control.cc)
target_link_libraries(move_control
    ${catkin_LIBRARIES} dock_control cogrob_navigation_route_protos glog gflags)

add_executable(cogrob_navigation_route_node src/cogrob_navigation_route_node.cc)
target_link_libraries(cogrob_navigation_route_node
    ${catkin_LIBRARIES} move_control glog gflags
    cogrob_navigation_route_workspace_cc_lib)
