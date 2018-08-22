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

#include <cmath>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "third_party/gflags.h"

DEFINE_double(lower_limit, 0.01, "Lower limit of a valid laser scan value.");
DEFINE_double(upper_limit, 4, "Upper limit of a valid laser scan value.");
DEFINE_double(replace_value, 4, "The value to replace.");
DEFINE_double(angle_error_step, 0.003, "To introduce some errors.");
DEFINE_string(change_frame, "", "Change the header to new value.");
DEFINE_int64(pub_every_n, 1, "Throttle.");

namespace {
ros::Publisher* pub;
double current_error = 0;
int throttle_index = 0;

void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  ++throttle_index;
  if (throttle_index >= FLAGS_pub_every_n) {
    throttle_index = 0;
  }
  if (throttle_index != 0) {
    return;
  }

  sensor_msgs::LaserScan new_msg = *msg;
  if (current_error > msg->angle_increment) {
    current_error = -msg->angle_increment;
  }

  if (FLAGS_change_frame != std::string("")) {
    new_msg.header.frame_id = FLAGS_change_frame;
  }

  new_msg.angle_min += current_error;
  new_msg.angle_max += current_error;
  current_error += FLAGS_angle_error_step;

  for(size_t i = 0; i < new_msg.ranges.size(); ++i) {
    if (!std::isfinite(new_msg.ranges[i])
        || (new_msg.ranges[i] < FLAGS_lower_limit)
        || (new_msg.ranges[i] > FLAGS_upper_limit)) {
      new_msg.ranges[i] = FLAGS_replace_value;
    }
  }
  if (pub) {
    pub->publish(new_msg);
  }
}
}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "laser_repub");
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  ros::NodeHandle ros_node;
  ros::Subscriber laser_sub = ros_node.subscribe("scan", 100, LaserCallback);
  ros::Publisher laser_repub = ros_node.advertise<sensor_msgs::LaserScan>(
      "scan_repub", 100);
  pub = &laser_repub;
  ros::spin();
}
