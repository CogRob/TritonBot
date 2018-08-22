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

#include "cogrob/universal_logger/event_entry_helper.h"

#include "absl/memory/memory.h"

namespace cogrob {
namespace universal_logger {

EventEntryHelper::EventEntryHelper(UniversalLoggerInterface* logger) {
  logger_ = logger;
  entry_ = absl::make_unique<ArchiveEntry>();
  SetStartTimestampToNow();
}

EventEntryHelper::EventEntryHelper(UniversalLoggerInterface* logger,
    std::unique_ptr<ArchiveEntry> entry) {
  logger_ = logger;
  entry_ = std::move(entry);
  SetStartTimestampToNow();
}

EventEntryHelper::EventEntryHelper(EventEntryHelper&& source) {
  logger_ = source.logger_;
  entry_ = std::move(source.entry_);
  SetStartTimestampToNow();
}

EventEntryHelper& EventEntryHelper::operator=(EventEntryHelper&& source) {
  logger_ = source.logger_;
  entry_ = std::move(source.entry_);
  SetStartTimestampToNow();
  return *this;
}

EventEntryHelper::~EventEntryHelper() {
  if (!entry_->event_metadata().has_end_timestamp()) {
    SetEndTimestampToNow();
  }
  double start_time = util::TimestampProtoToDouble(
      entry_->event_metadata().start_timestamp());
  double end_time = util::TimestampProtoToDouble(
      entry_->event_metadata().end_timestamp());
  entry_->mutable_event_metadata()->set_elapsed_time_sec(end_time - start_time);
  logger_->Log(std::move(*entry_));
}

void EventEntryHelper::SetStartTimestamp(
    const util::proto::Timestamp& timestamp_pb) {
  entry_->mutable_event_metadata()->mutable_start_timestamp()->CopyFrom(
      timestamp_pb);
}

void EventEntryHelper::SetStartTimestampToNow() {
  util::PopulateCurrentTimestamp(
      entry_->mutable_event_metadata()->mutable_start_timestamp());
}

void EventEntryHelper::SetEndTimestamp(
    const util::proto::Timestamp& timestamp_pb) {
  entry_->mutable_event_metadata()->mutable_start_timestamp()->CopyFrom(
      timestamp_pb);
}

void EventEntryHelper::SetEndTimestampToNow() {
  util::PopulateCurrentTimestamp(
      entry_->mutable_event_metadata()->mutable_end_timestamp());
}

void EventEntryHelper::SetIsSuccess(bool is_success) {
    entry_->mutable_event_metadata()->set_is_success(is_success);
}

ArchiveEntry* EventEntryHelper::GetMutableLogEntry() {
  return entry_.get();
}
const ArchiveEntry& EventEntryHelper::GetLogEntryRef() {
  return *entry_;
}

}  // namespace universal_logger
}  // namespace cogrob
