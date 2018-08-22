// Copyright (c) 2018, The Regents of the University of California
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
// * Neither the name of the University of California nor the
//   names of its contributors may be used to endorse or promote products
//   derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS OF THE UNIVERSITY OF CALIFORNIA
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <memory>
#include <utility>

#include "absl/memory/memory.h"
#include "cogrob/universal_logger/universal_logger.h"
#include "cogrob/universal_logger/universal_logger_flusher.h"
#include "cogrob/universal_logger/universal_logger_interface.h"
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "third_party/gflags.h"
#include "third_party/glog.h"

using cogrob::universal_logger::UniversalLogger;
using cogrob::universal_logger::UniversalLoggerFlusher;
using cogrob::universal_logger::UniversalLoggerInterface;

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "wakeword_logger_node");

  UniversalLoggerFlusher logger_flusher;
  auto universal_logger =
      absl::make_unique<UniversalLogger>("dialogue/wakeword");
  CHECK_OK(universal_logger->RegisterWithFlusher(&logger_flusher));

  ros::NodeHandle ros_node;

  auto goal_callback = [&universal_logger] (
      const std_msgs::Header::ConstPtr& msg) {
    cogrob::universal_logger::ArchiveEntry entry;
    entry.mutable_wakeword()->mutable_timestamp()->set_seconds(msg->stamp.sec);
    entry.mutable_wakeword()->mutable_timestamp()->set_nanos(msg->stamp.nsec);
    entry.mutable_wakeword()->set_text("snowboy");
    universal_logger->Log(std::move(entry));
  };
  ros::Subscriber goal_sub = ros_node.subscribe<std_msgs::Header>(
      "/cogrob/snowboy", 10, goal_callback);

  ros::spin();
  return 0;
}
