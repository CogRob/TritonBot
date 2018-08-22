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

#ifndef COGROB_UNIVERSAL_LOGGER_EVENT_ENTRY_HELPER_H_
#define COGROB_UNIVERSAL_LOGGER_EVENT_ENTRY_HELPER_H_

#include "cogrob/universal_logger/event_entry_helper_interface.h"

namespace cogrob {
namespace universal_logger {

// Automatically populates event metadata when created, and automatically write
// to logger when destoried.

class EventEntryHelper : public EventEntryHelperInterface {
 public:
  explicit EventEntryHelper(UniversalLoggerInterface* logger);
  explicit EventEntryHelper(UniversalLoggerInterface* logger,
      std::unique_ptr<ArchiveEntry> entry);
  EventEntryHelper(EventEntryHelper&& source);
  EventEntryHelper& operator=(EventEntryHelper&& source);

  EventEntryHelper(const EventEntryHelper& source) = delete;
  EventEntryHelper& operator=(const EventEntryHelper&) = delete;

  ~EventEntryHelper();

  void SetStartTimestamp(const util::proto::Timestamp& timestamp_pb) override;
  void SetStartTimestampToNow() override;
  void SetEndTimestamp(const util::proto::Timestamp& timestamp_pb) override;
  void SetEndTimestampToNow();
  void SetIsSuccess(bool is_success) override;
  ArchiveEntry* GetMutableLogEntry() override;
  const ArchiveEntry& GetLogEntryRef() override;
 private:
  UniversalLoggerInterface* logger_;
  std::unique_ptr<ArchiveEntry> entry_;
};

}  // namespace universal_logger
}  // namespace cogrob

#endif  // COGROB_UNIVERSAL_LOGGER_EVENT_ENTRY_HELPER_H_
