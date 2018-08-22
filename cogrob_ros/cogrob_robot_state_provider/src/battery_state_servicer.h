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

#ifndef COGROB_ROBOT_STATE_PROVIDER_BATTERY_STATE_SERVICER_H_
#define COGROB_ROBOT_STATE_PROVIDER_BATTERY_STATE_SERVICER_H_

#include <chrono>

#include "absl/synchronization/mutex.h"
#include "cogrob/robot_status/proto/battery_status.grpc.pb.h"
#include "cogrob/robot_status/proto/robot_status.grpc.pb.h"
#include "cogrob_robot_state_msgs/GetBatteryStatus.h"
#include "power_msgs/BatteryState.h"
#include "ros/ros.h"

#include "util/status.h"

namespace cogrob_robot_state_provider {

class BatteryStateServicer {
 public:
  BatteryStateServicer(ros::NodeHandle* ros_node);
  void BatteryStateCallback(const power_msgs::BatteryState::ConstPtr& msg);
  bool GetBatteryStatusServiceCallbak(
      cogrob_robot_state_msgs::GetBatteryStatus::Request& request,
      cogrob_robot_state_msgs::GetBatteryStatus::Response& response);
  util::Status GetBatteryStatusGrpc(
      const cogrob::robot_status::GetBatteryStatusRequest& request,
      cogrob::robot_status::GetBatteryStatusResponse* response);
 private:
  absl::Mutex mutex_;
  bool is_charging_ GUARDED_BY(mutex_);
  double charge_level_ GUARDED_BY(mutex_);
  std::chrono::time_point<std::chrono::system_clock>
      received_time_ GUARDED_BY(mutex_);
  std::unique_ptr<ros::Subscriber> battery_state_sub_;
  std::unique_ptr<ros::ServiceServer> get_battery_state_srv_;
};

}  // namespace cogrob_robot_state_provider

#endif  // COGROB_ROBOT_STATE_PROVIDER_BATTERY_STATE_SERVICER_H_
