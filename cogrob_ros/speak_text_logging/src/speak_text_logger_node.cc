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

#include "absl/memory/memory.h"
#include "cogrob/universal_logger/universal_logger.h"
#include "cogrob/universal_logger/universal_logger_flusher.h"
#include "cogrob/universal_logger/universal_logger_interface.h"
#include "ros/ros.h"
#include "speak_text_msgs/SpeakTextActionGoal.h"
#include "speak_text_msgs/SpeakTextActionResult.h"
#include "third_party/gflags.h"
#include "third_party/glog.h"

#include "speak_text_logger.h"

using speak_text_logging::SpeakTextLogger;
using cogrob::universal_logger::UniversalLogger;
using cogrob::universal_logger::UniversalLoggerFlusher;
using cogrob::universal_logger::UniversalLoggerInterface;

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "speak_text_logger_node");

  UniversalLoggerFlusher logger_flusher;
  auto universal_logger =
      absl::make_unique<UniversalLogger>("dialogue/robot_speech");
  CHECK_OK(universal_logger->RegisterWithFlusher(&logger_flusher));
  SpeakTextLogger speak_text_logger(std::move(universal_logger));

  ros::NodeHandle ros_node;

  auto goal_callback = [&speak_text_logger] (
      const speak_text_msgs::SpeakTextActionGoal::ConstPtr& msg) {
    speak_text_logger.ActionGoalCallback(msg);
  };
  ros::Subscriber goal_sub =
      ros_node.subscribe<speak_text_msgs::SpeakTextActionGoal>(
      "/cogrob/speak_text_action/goal", 10, goal_callback);

  auto result_callback = [&speak_text_logger] (
      const speak_text_msgs::SpeakTextActionResult::ConstPtr& msg) {
    speak_text_logger.ActionResultCallback(msg);
  };
  ros::Subscriber result_sub =
      ros_node.subscribe<speak_text_msgs::SpeakTextActionResult>(
      "/cogrob/speak_text_action/result", 10, result_callback);

  ros::spin();
  return 0;
}
