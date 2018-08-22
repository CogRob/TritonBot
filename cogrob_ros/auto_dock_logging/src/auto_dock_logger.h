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

#ifndef AUTO_DOCK_LOGGER_H
#define AUTO_DOCK_LOGGER_H

#include <unordered_map>

#include "absl/synchronization/mutex.h"
#include "actionlib_msgs/GoalID.h"
#include "cogrob/universal_logger/universal_logger.h"
#include "cogrob/universal_logger/proto/archive_entry.pb.h"
#include "fetch_auto_dock_msgs/DockActionGoal.h"
#include "fetch_auto_dock_msgs/DockActionResult.h"
#include "fetch_auto_dock_msgs/UndockActionGoal.h"
#include "fetch_auto_dock_msgs/UndockActionResult.h"
#include "goal_id_hash.h"
#include "ros/ros.h"

namespace auto_dock_logging {

class GetRobotPositionProxy {
 public:
  GetRobotPositionProxy(ros::ServiceClient* service_client);
  bool GetRobotPosition(double* x, double* y, double* orientation);
 private:
  ros::ServiceClient* service_client_;
};

class AutoDockLogger {
 public:
  AutoDockLogger(
      cogrob::universal_logger::UniversalLoggerInterface* universal_logger,
      GetRobotPositionProxy* get_robot_position_proxy);
  ~AutoDockLogger();
  void DockActionGoalCallback(
      const fetch_auto_dock_msgs::DockActionGoal::ConstPtr& msg);
  void DockActionResultCallback(
      const fetch_auto_dock_msgs::DockActionResult::ConstPtr& msg);
  void UndockActionGoalCallback(
      const fetch_auto_dock_msgs::UndockActionGoal::ConstPtr& msg);
  void UndockActionResultCallback(
      const fetch_auto_dock_msgs::UndockActionResult::ConstPtr& msg);

 private:
  // Stores active log entries. Activates when receive a new goal, and will
  // retire when received a result or after certain timeout.
  std::unordered_map<
    actionlib_msgs::GoalID,
    std::unique_ptr<cogrob::universal_logger::ArchiveEntry>> log_entry_;
  absl::Mutex log_entry_mutex_;

  cogrob::universal_logger::UniversalLoggerInterface* universal_logger_;
  GetRobotPositionProxy* get_robot_position_proxy_;
};

}  // namespace auto_dock_logging

#endif  // AUTO_DOCK_LOGGER_H
