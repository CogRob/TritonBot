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

#include "acoustic_magic_msgs/SoundSourceAngle.h"
#include "cogrob/hardware/acoustic_magic/uart_reader.h"
#include "ros/ros.h"
#include "third_party/gflags.h"
#include "third_party/glog.h"
#include "util/statusor.h"

using cogrob::hardware::acoustic_magic::UartQueue;
using cogrob::hardware::acoustic_magic::UartReader;

DEFINE_string(acoustic_magic_uart_device, "/dev/ttyUSB0",
    "Path to Acoustic Magic USB device.");

int main(int argc, char *argv[])
{
  google::InitGoogleLogging(argv[0]);
  google::LogToStderr();
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "acoustic_magic_uart");

  UartQueue uart_queue;
  UartReader reader(FLAGS_acoustic_magic_uart_device, &uart_queue);

  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<acoustic_magic_msgs::SoundSourceAngle>(
      "cogrob/microphone_direction", 10);

  uint32_t seq = 0;

  while (true) {
    // Blocking pop, 1000 msec.
    util::StatusOr<uint8_t> val = uart_queue.blocking_pop(1000);
    // If it fails, kill the program.
    uint8_t raw_val = val.ValueOrDie();
    LOG(INFO) << static_cast<int>(raw_val);
    acoustic_magic_msgs::SoundSourceAngle msg;
    msg.header.stamp = ros::Time::now();
    msg.header.seq = seq++;

    if (raw_val == 255) {
      msg.is_valid = false;
      msg.angle = 0;
    } else {
      LOG_IF(ERROR, raw_val > 250) << "Value greater than 250 but not 255, "
                                   << "raw_val is " << raw_val;
      msg.is_valid = true;
      msg.angle = (125.0 - raw_val) / 250.0 * 180.0;
    }
    pub.publish(msg);
  }

  return 0;
}
