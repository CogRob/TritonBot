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
#include "gcloud_speech_logger.h"
#include "gcloud_speech_msgs/LinearPcm16Le16000Audio.h"
#include "gcloud_speech_msgs/SpeechToTextActionFeedback.h"
#include "gcloud_speech_msgs/SpeechToTextActionGoal.h"
#include "gcloud_speech_msgs/SpeechToTextActionResult.h"
#include "third_party/glog.h"


namespace gcloud_speech_logging {

GcloudSpeechLogger::GcloudSpeechLogger(
    std::unique_ptr<cogrob::universal_logger::UniversalLoggerInterface>
    universal_logger) {
  absl::MutexLock lock(&log_entry_mutex_);
  universal_logger_ = std::move(universal_logger);
}

GcloudSpeechLogger::~GcloudSpeechLogger() {
  absl::MutexLock lock(&log_entry_mutex_);
}

void GcloudSpeechLogger::ActionGoalCallback(
    const gcloud_speech_msgs::SpeechToTextActionGoal::ConstPtr& msg) {
  absl::MutexLock lock(&log_entry_mutex_);
  // We have received a new goal.
  if (log_entry_.count(msg->goal_id)) {
    LOG(ERROR) << "Received a goal that is alrady active, id is "
               << msg->goal_id.id;
  } else {
    log_entry_[msg->goal_id] =
        absl::make_unique<cogrob::dialogue::logging::VoiceRecognitionLog>();
    LOG(INFO) << "Logging enabled for goal: " << msg->goal_id.id;

    // Uses timestamp in GoalID to set start_timestamp.
    log_entry_[msg->goal_id]->mutable_start_timestamp()->set_seconds(
        msg->goal_id.stamp.sec);
    log_entry_[msg->goal_id]->mutable_start_timestamp()->set_nanos(
        msg->goal_id.stamp.nsec);

    // Logs information for starting the goal.
    for (const auto& hint : msg->goal.hints) {
      log_entry_[msg->goal_id]->add_hints(hint);
    }
    log_entry_[msg->goal_id]->set_max_alternatives(msg->goal.max_alternatives);
    log_entry_[msg->goal_id]->set_listen_duration_sec(
        msg->goal.listen_duration_sec);
    log_entry_[msg->goal_id]->set_max_recognition_duration_sec(
        msg->goal.max_recognition_duration_sec);
    log_entry_[msg->goal_id]->set_suppress_interim_results(
        msg->goal.suppress_interim_results);
  }
}

void GcloudSpeechLogger::ActionFeedbackCallback(
    const gcloud_speech_msgs::SpeechToTextActionFeedback::ConstPtr& msg) {
  absl::MutexLock lock(&log_entry_mutex_);
  if (!log_entry_.count(msg->status.goal_id)) {
    LOG(ERROR) << "Received feedback of a goal that is not active for logging, "
               << "id: " << msg->status.goal_id.id;
  } else {
    LOG(INFO) << "Logging feedback for id: " << msg->status.goal_id.id;
    for (const auto& hypothesis: msg->feedback.hypotheses) {
      auto* new_intrim_result =
          log_entry_[msg->status.goal_id]->add_intrim_results();
      new_intrim_result->set_received_time_sec(
        (msg->header.stamp - msg->status.goal_id.stamp).toSec());
      new_intrim_result->set_transcript(hypothesis.transcript);
      new_intrim_result->set_confidence(hypothesis.confidence);
      new_intrim_result->set_stability(msg->feedback.stability);
      new_intrim_result->set_is_final(msg->feedback.is_portion_final);
      LOG(INFO) << "Logged: " << new_intrim_result->transcript();
    }
  }
}

void GcloudSpeechLogger::ActionResultCallback(
    const gcloud_speech_msgs::SpeechToTextActionResult::ConstPtr& msg) {
  absl::MutexLock lock(&log_entry_mutex_);
  if (!log_entry_.count(msg->status.goal_id)) {
    LOG(ERROR) << "Received result of a goal that is not active for logging, "
               << "id: " << msg->status.goal_id.id;
  } else {
    LOG(INFO) << "Logging result for goal id: " << msg->status.goal_id.id;
    log_entry_[msg->status.goal_id]->set_final_results(msg->result.transcript);

    // Finish and remove from map.
    cogrob::universal_logger::ArchiveEntry entry;
    entry.set_allocated_voice_recognition(
        log_entry_[msg->status.goal_id].release());
    log_entry_.erase(msg->status.goal_id);
    universal_logger_->Log(std::move(entry));
    // TODO(shengye): flush universal_logger_;
  }
}

void GcloudSpeechLogger::MicrophoneAudioCallback(
    const gcloud_speech_msgs::LinearPcm16Le16000Audio::ConstPtr& msg) {
  absl::MutexLock lock(&log_entry_mutex_);
  if (log_entry_.size() > 1) {
    LOG(WARNING) << "There are " << log_entry_.size() << " active goals. "
                 << "The logger will work normally, but this may indicate a "
                 << "problem in the rest of the system.";
  }

  // Insert audio recording to all of the active logging goals.
  for (auto& kv : log_entry_) {
    kv.second->mutable_audio_recording()->insert(
        std::end(*(kv.second->mutable_audio_recording())),
        std::begin(msg->data), std::end(msg->data));
  }
}

}  // namespace gcloud_speech_logging
