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

#include <atomic>
#include <memory>

#include "absl/memory/memory.h"
#include "absl/synchronization/mutex.h"
#include "actionlib_msgs/GoalID.h"
#include "cogrob/universal_logger/universal_logger.h"
#include "cogrob_robot_state_msgs/GetRobotPosition.h"
#include "auto_dock_logger.h"
#include "fetch_auto_dock_msgs/DockActionGoal.h"
#include "fetch_auto_dock_msgs/DockActionResult.h"
#include "fetch_auto_dock_msgs/UndockActionGoal.h"
#include "fetch_auto_dock_msgs/UndockActionResult.h"
#include "third_party/glog.h"
#include "util/timestamp.h"

namespace auto_dock_logging {

AutoDockLogger::AutoDockLogger(
    cogrob::universal_logger::UniversalLoggerInterface* universal_logger,
    GetRobotPositionProxy* get_robot_position_proxy) {
  absl::MutexLock lock(&log_entry_mutex_);
  universal_logger_ = universal_logger;
  get_robot_position_proxy_ = get_robot_position_proxy;
}

AutoDockLogger::~AutoDockLogger() {
  absl::MutexLock lock(&log_entry_mutex_);
}

void AutoDockLogger::DockActionGoalCallback(
    const fetch_auto_dock_msgs::DockActionGoal::ConstPtr& msg) {
  absl::MutexLock lock(&log_entry_mutex_);
  // We have received a new goal.
  if (log_entry_.count(msg->goal_id)) {
    LOG(ERROR) << "Received a goal that is alrady active, id is "
               << msg->goal_id.id;
  } else {
    log_entry_[msg->goal_id] =
        absl::make_unique<cogrob::universal_logger::ArchiveEntry>();
    LOG(INFO) << "Logging enabled for goal: " << msg->goal_id.id;

    // Uses timestamp in GoalID to set start_timestamp.
    log_entry_[msg->goal_id]->mutable_event_metadata()->
        mutable_start_timestamp()->set_seconds(msg->goal_id.stamp.sec);
    log_entry_[msg->goal_id]->mutable_event_metadata()->
        mutable_start_timestamp()->set_nanos(msg->goal_id.stamp.nsec);

    // Logs information for starting the goal.
    log_entry_[msg->goal_id]->mutable_auto_dock()->set_dock_type(
        cogrob::navigation::AutoDockLog::DOCK);
    double x = 0, y = 0, orientation = 0;
    get_robot_position_proxy_->GetRobotPosition(&x, &y, &orientation);
    log_entry_[msg->goal_id]->mutable_auto_dock()->
        mutable_start_world_pos()->set_x(x);
    log_entry_[msg->goal_id]->mutable_auto_dock()->
        mutable_start_world_pos()->set_y(y);
    log_entry_[msg->goal_id]->mutable_auto_dock()->
        mutable_start_world_pos()->set_orientation(orientation);
  }
}

void AutoDockLogger::UndockActionGoalCallback(
    const fetch_auto_dock_msgs::UndockActionGoal::ConstPtr& msg) {
  absl::MutexLock lock(&log_entry_mutex_);
  // We have received a new goal.
  if (log_entry_.count(msg->goal_id)) {
    LOG(ERROR) << "Received a goal that is alrady active, id is "
               << msg->goal_id.id;
  } else {
    log_entry_[msg->goal_id] =
        absl::make_unique<cogrob::universal_logger::ArchiveEntry>();
    LOG(INFO) << "Logging enabled for goal: " << msg->goal_id.id;

    // Uses timestamp in GoalID to set start_timestamp.
    log_entry_[msg->goal_id]->mutable_event_metadata()->
        mutable_start_timestamp()->set_seconds(msg->goal_id.stamp.sec);
    log_entry_[msg->goal_id]->mutable_event_metadata()->
        mutable_start_timestamp()->set_nanos(msg->goal_id.stamp.nsec);

    // Logs information for starting the goal.
    log_entry_[msg->goal_id]->mutable_auto_dock()->set_dock_type(
        cogrob::navigation::AutoDockLog::UNDOCK);
    double x = 0, y = 0, orientation = 0;
    get_robot_position_proxy_->GetRobotPosition(&x, &y, &orientation);
    log_entry_[msg->goal_id]->mutable_auto_dock()->
        mutable_start_world_pos()->set_x(x);
    log_entry_[msg->goal_id]->mutable_auto_dock()->
        mutable_start_world_pos()->set_y(y);
    log_entry_[msg->goal_id]->mutable_auto_dock()->
        mutable_start_world_pos()->set_orientation(orientation);
  }
}

void AutoDockLogger::DockActionResultCallback(
    const fetch_auto_dock_msgs::DockActionResult::ConstPtr& msg) {
  absl::MutexLock lock(&log_entry_mutex_);
  if (!log_entry_.count(msg->status.goal_id)) {
    LOG(ERROR) << "Received result of a goal that is not active for logging, "
               << "id: " << msg->status.goal_id.id;
  } else {
    LOG(INFO) << "Logging result for goal id: " << msg->status.goal_id.id;

    // Uses timestamp in GoalID to set start_timestamp.
    log_entry_[msg->status.goal_id]->mutable_event_metadata()->
        mutable_end_timestamp()->set_seconds(msg->header.stamp.sec);
    log_entry_[msg->status.goal_id]->mutable_event_metadata()->
        mutable_end_timestamp()->set_nanos(msg->header.stamp.nsec);
    log_entry_[msg->status.goal_id]->mutable_event_metadata()->
        set_elapsed_time_sec(util::TimestampProtoToDouble(
        log_entry_[msg->status.goal_id]->event_metadata().end_timestamp()) -
        util::TimestampProtoToDouble(log_entry_[msg->status.goal_id]->
        event_metadata().start_timestamp()));

    // Logs information for finishing the goal.
    if (msg->result.docked) {
      log_entry_[msg->status.goal_id]->mutable_auto_dock()->set_result(
          cogrob::navigation::SUCCESS_RESULT);
      log_entry_[msg->status.goal_id]->mutable_event_metadata()->
          set_is_success(true);
    } else {
      log_entry_[msg->status.goal_id]->mutable_auto_dock()->set_result(
          cogrob::navigation::FAILED_RESULT);
      log_entry_[msg->status.goal_id]->mutable_event_metadata()->
          set_is_success(false);
    }
    double x = 0, y = 0, orientation = 0;
    get_robot_position_proxy_->GetRobotPosition(&x, &y, &orientation);
    log_entry_[msg->status.goal_id]->mutable_auto_dock()
        ->mutable_end_world_pos()->set_x(x);
    log_entry_[msg->status.goal_id]->mutable_auto_dock()
        ->mutable_end_world_pos()->set_y(y);
    log_entry_[msg->status.goal_id]->mutable_auto_dock()
        ->mutable_end_world_pos()->set_orientation(orientation);

    // Finish and remove from map.
    std::unique_ptr<cogrob::universal_logger::ArchiveEntry> entry =
        std::move(log_entry_[msg->status.goal_id]);
    log_entry_.erase(msg->status.goal_id);
    universal_logger_->Log(std::move(*entry));
  }
}

void AutoDockLogger::UndockActionResultCallback(
    const fetch_auto_dock_msgs::UndockActionResult::ConstPtr& msg) {
  absl::MutexLock lock(&log_entry_mutex_);
  if (!log_entry_.count(msg->status.goal_id)) {
    LOG(ERROR) << "Received result of a goal that is not active for logging, "
               << "id: " << msg->status.goal_id.id;
  } else {
    LOG(INFO) << "Logging result for goal id: " << msg->status.goal_id.id;

    // Uses timestamp in GoalID to set start_timestamp.
    log_entry_[msg->status.goal_id]->mutable_event_metadata()->
        mutable_end_timestamp()->set_seconds(msg->header.stamp.sec);
    log_entry_[msg->status.goal_id]->mutable_event_metadata()->
        mutable_end_timestamp()->set_nanos(msg->header.stamp.nsec);
    log_entry_[msg->status.goal_id]->mutable_event_metadata()->
        set_elapsed_time_sec(util::TimestampProtoToDouble(
        log_entry_[msg->status.goal_id]->event_metadata().end_timestamp()) -
        util::TimestampProtoToDouble(log_entry_[msg->status.goal_id]->
        event_metadata().start_timestamp()));

    // Logs information for finishing the goal.
    if (msg->result.undocked) {
      log_entry_[msg->status.goal_id]->mutable_auto_dock()->set_result(
          cogrob::navigation::SUCCESS_RESULT);
      log_entry_[msg->status.goal_id]->mutable_event_metadata()->
          set_is_success(true);
    } else {
      log_entry_[msg->status.goal_id]->mutable_auto_dock()->set_result(
          cogrob::navigation::FAILED_RESULT);
      log_entry_[msg->status.goal_id]->mutable_event_metadata()->
          set_is_success(false);
    }
    double x = 0, y = 0, orientation = 0;
    get_robot_position_proxy_->GetRobotPosition(&x, &y, &orientation);
    log_entry_[msg->status.goal_id]->mutable_auto_dock()
        ->mutable_end_world_pos()->set_x(x);
    log_entry_[msg->status.goal_id]->mutable_auto_dock()
        ->mutable_end_world_pos()->set_y(y);
    log_entry_[msg->status.goal_id]->mutable_auto_dock()
        ->mutable_end_world_pos()->set_orientation(orientation);

    // Finish and remove from map.
    std::unique_ptr<cogrob::universal_logger::ArchiveEntry> entry =
        std::move(log_entry_[msg->status.goal_id]);
    log_entry_.erase(msg->status.goal_id);
    universal_logger_->Log(std::move(*entry));
  }
}

GetRobotPositionProxy::GetRobotPositionProxy(
    ros::ServiceClient* service_client) {
  service_client_ = service_client;
}

bool GetRobotPositionProxy::GetRobotPosition(
    double* x, double* y, double* orientation) {
  cogrob_robot_state_msgs::GetRobotPosition srv;
  if (!service_client_->call(srv)) {
    return false;
  }
  if (srv.response.is_error) {
    return false;
  }
  *x = srv.response.world_x;
  *y = srv.response.world_y;
  *orientation = srv.response.orientation;
}

// TODO(shengye): Extract common code between Dock and Undock, maybe using
// template?

}  // namespace auto_dock_logging
