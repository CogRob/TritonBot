// Copyright (c) 2017, The Regents of the University of California
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

#include "stdint.h"
#include <vector>

#include "ros/ros.h"
#include "gcloud_speech_msgs/LinearPcm16Le16000Audio.h"
#include "snowboy_wrapper.h"
#include "std_msgs/Header.h"
#include "third_party/glog.h"
#include "third_party/gflags.h"

DEFINE_string(snowboy_res_path, "", "Path to snowboy \"common.res\".");
DEFINE_string(snowboy_umdl_path, "", "Path to snowboy \"snowboy.umdl\".");
DEFINE_string(snowboy_sensitivity, "0.5", "Snowboy sensitivity directives.");

using snowboy::SnowboyWrapper;
using gcloud_speech_msgs::LinearPcm16Le16000Audio;

void MicrophoneCallback(
    const LinearPcm16Le16000Audio::ConstPtr& msg,
    SnowboyWrapper* snowboy_wrapper, ros::Publisher* publisher) {
  static uint32_t seq = 0;

  std::vector<int16_t> data(msg->data.size() / 2);
  for (size_t i = 0; i < data.size(); ++i) {
    data[i] = (static_cast<int16_t>(msg->data[i * 2 + 1]) << 8)
               | msg->data[i * 2];
  }
  int detect_result = snowboy_wrapper->RunDetection(data.data(), data.size());
  LOG(INFO) << "Detection result is " << detect_result;
  if (detect_result > 0) {
    std_msgs::Header msg_pub;
    msg_pub.seq = ++seq;
    msg_pub.stamp = ros::Time::now();
    publisher->publish(msg_pub);
  }
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::LogToStderr();
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "snowboy_detector");

  SnowboyWrapper snowboy_wrapper(FLAGS_snowboy_res_path.c_str(),
                                 FLAGS_snowboy_umdl_path.c_str());
  CHECK_EQ(snowboy_wrapper.SampleRate(), 16000);
  CHECK_EQ(snowboy_wrapper.NumChannels(), 1);
  CHECK_EQ(snowboy_wrapper.BitsPerSample(), 16);
  snowboy_wrapper.SetSensitivity(FLAGS_snowboy_sensitivity.c_str());

  ros::NodeHandle ros_node;
  ros::Publisher snowboy_pub = ros_node.advertise<std_msgs::Header>(
      "/cogrob/snowboy", 10);
  ros::Subscriber mic_sub = ros_node.subscribe<LinearPcm16Le16000Audio>(
      "/cogrob/microphone_audio", 10,
      [&] (const LinearPcm16Le16000Audio::ConstPtr& msg) {
        MicrophoneCallback(msg, &snowboy_wrapper, &snowboy_pub);
      });

  ros::spin();
}
