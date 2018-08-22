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
project(speak_text_logging)

find_package(catkin REQUIRED COMPONENTS grpc speak_text_msgs abseil_cpp actionlib_msgs roscpp)

catkin_package()

file(GLOB_RECURSE all_proto_files src/*.proto)
generate_proto(speak_text_logging_protos
  SRC_BASE src/workspace GRPC ${all_proto_files})

add_executable(speak_text_logger_node
  src/goal_id_hash.cc
  src/speak_text_logger.cc
  src/speak_text_logger_node.cc
  src/workspace/cogrob/universal_logger/universal_logger.cc
  src/workspace/cogrob/universal_logger/universal_logger_flusher.cc
  # src/workspace/third_party/protobuf/gzip_stream.cc
  src/workspace/util/file_system.cc
  src/workspace/util/status.cc
  src/workspace/util/statusor.cc
  src/workspace/util/timestamp.cc
  src/workspace/util/uuid.cc
)

include_directories(src)
include_directories(src/workspace)

target_link_libraries(speak_text_logger_node
  ${catkin_LIBRARIES} speak_text_logging_protos glog gflags uuid)
