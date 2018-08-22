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
#include "speak_text_logger.h"
#include "speak_text_msgs/SpeakTextActionGoal.h"
#include "speak_text_msgs/SpeakTextActionResult.h"
#include "third_party/glog.h"


namespace speak_text_logging {

SpeakTextLogger::SpeakTextLogger(
    std::unique_ptr<cogrob::universal_logger::UniversalLoggerInterface>
    universal_logger) {
  absl::MutexLock lock(&log_entry_mutex_);
  universal_logger_ = std::move(universal_logger);
}

SpeakTextLogger::~SpeakTextLogger() {
  absl::MutexLock lock(&log_entry_mutex_);
}

void SpeakTextLogger::ActionGoalCallback(
    const speak_text_msgs::SpeakTextActionGoal::ConstPtr& msg) {
  absl::MutexLock lock(&log_entry_mutex_);
  // We have received a new goal.
  if (log_entry_.count(msg->goal_id)) {
    LOG(ERROR) << "Received a goal that is alrady active, id is "
               << msg->goal_id.id;
  } else {
    log_entry_[msg->goal_id] =
        absl::make_unique<cogrob::dialogue::logging::RobotSpeechLog>();
    LOG(INFO) << "Logging enabled for goal: " << msg->goal_id.id;

    // Uses timestamp in GoalID to set start_timestamp.
    log_entry_[msg->goal_id]->mutable_start_timestamp()->set_seconds(
        msg->goal_id.stamp.sec);
    log_entry_[msg->goal_id]->mutable_start_timestamp()->set_nanos(
        msg->goal_id.stamp.nsec);

    // Logs information for starting the goal.
    log_entry_[msg->goal_id]->set_text(msg->goal.text);
  }
}

void SpeakTextLogger::ActionResultCallback(
    const speak_text_msgs::SpeakTextActionResult::ConstPtr& msg) {
  absl::MutexLock lock(&log_entry_mutex_);
  if (!log_entry_.count(msg->status.goal_id)) {
    LOG(ERROR) << "Received result of a goal that is not active for logging, "
               << "id: " << msg->status.goal_id.id;
  } else {
    LOG(INFO) << "Logging result for goal id: " << msg->status.goal_id.id;

    // Uses timestamp in GoalID to set start_timestamp.
    log_entry_[msg->status.goal_id]->mutable_end_timestamp()->set_seconds(
        msg->header.stamp.sec);
    log_entry_[msg->status.goal_id]->mutable_end_timestamp()->set_nanos(
        msg->header.stamp.nsec);

    log_entry_[msg->status.goal_id]->set_total_time(msg->result.total_time);
    log_entry_[msg->status.goal_id]->set_is_success(msg->result.is_success);
    log_entry_[msg->status.goal_id]->set_error_info(msg->result.error_info);

    // Finish and remove from map.
    cogrob::universal_logger::ArchiveEntry entry;
    entry.set_allocated_robot_speech(
        log_entry_[msg->status.goal_id].release());
    log_entry_.erase(msg->status.goal_id);
    universal_logger_->Log(std::move(entry));
  }
}

}  // namespace speak_text_logging
