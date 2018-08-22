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

#include "battery_state_servicer.h"

#include "absl/memory/memory.h"
#include "absl/synchronization/mutex.h"
#include "util/status.h"

namespace cogrob_robot_state_provider {

BatteryStateServicer::BatteryStateServicer(ros::NodeHandle* ros_node) {
  auto battery_state_callback = [this] (
    const power_msgs::BatteryState::ConstPtr& msg) {
    this->BatteryStateCallback(msg);
  };
  battery_state_sub_ = absl::make_unique<ros::Subscriber>(
      ros_node->subscribe<power_msgs::BatteryState>(
      "/battery_state", 10, battery_state_callback));

  auto get_battery_status_service = [this] (
    cogrob_robot_state_msgs::GetBatteryStatus::Request& request,
    cogrob_robot_state_msgs::GetBatteryStatus::Response& response) -> bool {
    return this->GetBatteryStatusServiceCallbak(request, response);
  };
  get_battery_state_srv_ = absl::make_unique<ros::ServiceServer>(
      ros_node->advertiseService<
      cogrob_robot_state_msgs::GetBatteryStatus::Request,
      cogrob_robot_state_msgs::GetBatteryStatus::Response>(
      "/cogrob/get_battery_status", get_battery_status_service));
}

void BatteryStateServicer::BatteryStateCallback(
    const power_msgs::BatteryState::ConstPtr& msg) {
  absl::WriterMutexLock lock(&mutex_);
  charge_level_ = msg->charge_level;
  is_charging_ = msg->is_charging;
  received_time_ = std::chrono::system_clock::now();
}

bool BatteryStateServicer::GetBatteryStatusServiceCallbak(
    cogrob_robot_state_msgs::GetBatteryStatus::Request& request,
    cogrob_robot_state_msgs::GetBatteryStatus::Response& response) {
  absl::ReaderMutexLock lock(&mutex_);
  response.is_charging = is_charging_;
  response.charge_level = charge_level_;
  const auto /* std::chrono::duration */ age_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now() - received_time_).count();
  // TODO(shengye): Use gflags for this.
  if (age_us > 5000000) {
    response.is_error = true;
    response.error_message = "Battery state expired.";
  }
  return true;
}

util::Status BatteryStateServicer::GetBatteryStatusGrpc(
    const cogrob::robot_status::GetBatteryStatusRequest& request,
    cogrob::robot_status::GetBatteryStatusResponse* response) {
  absl::ReaderMutexLock lock(&mutex_);
  auto* battery_status = response->mutable_battery_status();

  battery_status->set_is_charging(is_charging_);
  battery_status->set_charge_level_percentage(charge_level_);
  // TODO(shengye): Provide voltages for each cells.

  const auto /* std::chrono::duration */ age_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now() - received_time_).count();
  // TODO(shengye): Use gflags for this.
  if (age_us > 5000000) {
    return util::Status(util::error::INTERNAL, "Battery state expired.");
  }
  return util::Status::OK;
}

}  // namespace cogrob_robot_state_provider
