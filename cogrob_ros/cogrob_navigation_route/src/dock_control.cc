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

#include "dock_control.h"

#include <cmath>

#include "third_party/glog.h"

namespace cogrob_navigation_route {

DockControl::DockControl(ros::NodeHandle* ros_node) {
  dock_action_ = absl::make_unique<actionlib::SimpleActionClient<
      fetch_auto_dock_msgs::DockAction>>("/dock", true);
  dock_action_->waitForServer();
  LOG(INFO) << "Dock action client ready.";

  undock_action_ = absl::make_unique<actionlib::SimpleActionClient<
      fetch_auto_dock_msgs::UndockAction>>("/undock", true);
  undock_action_->waitForServer();
  LOG(INFO) << "Undock action client ready.";

  robot_position_client_ = absl::make_unique<ros::ServiceClient>(
      ros_node->serviceClient<cogrob_robot_state_msgs::GetRobotPosition>(
      "/cogrob/get_robot_position"));
  LOG(INFO) << "Robot position service ready.";
}

util::StatusOr<bool> DockControl::Dock() {
  fetch_auto_dock_msgs::DockGoal goal;
  dock_action_->sendGoal(goal);
  dock_action_->waitForResult(ros::Duration(60.0));
  if (dock_action_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    return dock_action_->getResult()->docked;
  }
  return util::Status(util::error::DEADLINE_EXCEEDED,
                      "Dock took more than 60 secs or failed prematurely");
}

util::StatusOr<bool> DockControl::Undock() {
  fetch_auto_dock_msgs::UndockGoal goal;
  undock_action_->sendGoal(goal);
  undock_action_->waitForResult(ros::Duration(60.0));
  if (undock_action_->getState()
      == actionlib::SimpleClientGoalState::SUCCEEDED) {
    return undock_action_->getResult()->undocked;
  }
  return util::Status(util::error::DEADLINE_EXCEEDED,
                      "Undock took more than 60 secs or failed prematurely");
}

util::StatusOr<bool> DockControl::UndockIfNearHome() {
  cogrob_robot_state_msgs::GetRobotPosition robot_position_srv;
  if (!robot_position_client_->call(robot_position_srv)) {
    LOG(ERROR) << "GetRobotPosition failed.";
    return util::Status(util::error::UNAVAILABLE, "GetRobotPosition failed.");
  }

  double world_x = robot_position_srv.response.world_x;
  double world_y = robot_position_srv.response.world_y;
  double orientation = robot_position_srv.response.orientation;

  const double PI = 3.14159265358979323846;
  if (std::abs(world_x) < 0.2 and std::abs(world_y) < 0.2 and
      std::abs(orientation) < PI * 0.2) {
    return Undock();
  } else {
    // Not in the docking range, return true.
    // TODO(shengye): Test if charging, return false if charging.
    return true;
  }
}

}  // namespace cogrob_navigation_route
