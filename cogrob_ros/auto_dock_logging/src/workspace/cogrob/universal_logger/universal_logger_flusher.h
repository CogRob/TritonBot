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

#ifndef COGROB_UNIVERSAL_LOGGER_UNIVERSAL_LOGGER_FLUSHER_H_
#define COGROB_UNIVERSAL_LOGGER_UNIVERSAL_LOGGER_FLUSHER_H_

#include <unordered_set>
#include <thread>

#include "absl/synchronization/mutex.h"
#include "cogrob/universal_logger/universal_logger_flusher_interface.h"
#include "third_party/gflags.h"
#include "util/status.h"

DECLARE_int32(log_flush_interval_sec);

namespace cogrob {
namespace universal_logger {

class UniversalLoggerFlusher : public UniversalLoggerFlusherInterface {
 public:
  UniversalLoggerFlusher();
  ~UniversalLoggerFlusher();
  util::Status Register(UniversalLoggerInterface*) override;
  util::Status Unregister(UniversalLoggerInterface*) override;

  util::Status FlushAll() override;
 private:
  void PeriodicFlushThread();

  absl::Mutex managed_loggers_mutex_;
  std::unordered_set<UniversalLoggerInterface*> managed_loggers_;

  absl::Mutex periodic_flush_mutex_;
  bool stop_periodic_flush_thread_ = false;
  std::thread periodic_flush_thread_;
};

}  // namespace universal_logger
}  // namespace cogrob

#endif  // COGROB_UNIVERSAL_LOGGER_UNIVERSAL_LOGGER_FLUSHER_H_
