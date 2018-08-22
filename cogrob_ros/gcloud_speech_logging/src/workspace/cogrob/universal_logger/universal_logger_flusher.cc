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

#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "cogrob/universal_logger/universal_logger_flusher.h"
#include "third_party/gflags.h"
#include "third_party/glog.h"

DEFINE_int32(log_flush_interval_sec, 60,
             "Internal to flush logs in UniversalLogger, in seconds.");

namespace cogrob {
namespace universal_logger {

UniversalLoggerFlusher::UniversalLoggerFlusher() {
  periodic_flush_thread_ = std::thread([&] { PeriodicFlushThread(); });
}

UniversalLoggerFlusher::~UniversalLoggerFlusher() {
  FlushAll();
  {
    absl::MutexLock lock(&periodic_flush_mutex_);
    stop_periodic_flush_thread_ = true;
  }
  periodic_flush_thread_.join();
  CHECK_EQ(managed_loggers_.size(), 0);
}

util::Status UniversalLoggerFlusher::Register(
    UniversalLoggerInterface* logger) {
  absl::MutexLock lock(&managed_loggers_mutex_);
  LOG_IF(FATAL, logger == nullptr) << "Registering a nullptr.";

  if (managed_loggers_.count(logger)) {
    return util::Status(
        util::error::FAILED_PRECONDITION, "Already registered.");
  }
  managed_loggers_.insert(logger);
  return util::Status::OK;
}

util::Status UniversalLoggerFlusher::Unregister(
    UniversalLoggerInterface* logger) {
  absl::MutexLock lock(&managed_loggers_mutex_);
  LOG_IF(FATAL, logger == nullptr) << "Unregistering a nullptr.";

  if (!managed_loggers_.count(logger)) {
    return util::Status(
        util::error::FAILED_PRECONDITION, "Not registered.");
  }
  managed_loggers_.erase(logger);
  return util::Status::OK;
}

util::Status UniversalLoggerFlusher::FlushAll() {
  absl::MutexLock lock(&managed_loggers_mutex_);
  util::Status result = util::Status::OK;
  for (auto* logger : managed_loggers_) {
    util::Status current_result = logger->Flush();
    if (!current_result.ok()) {
      result = current_result;
    }
  }
  return result;
}

void UniversalLoggerFlusher::PeriodicFlushThread() {
  periodic_flush_mutex_.Lock();
  while (true) {
    if (periodic_flush_mutex_.AwaitWithTimeout(
        absl::Condition(&stop_periodic_flush_thread_),
        absl::Seconds(FLAGS_log_flush_interval_sec))) {
      // The condition is true, which means stop is requested.
      break;
    }
    FlushAll();
  }
  periodic_flush_mutex_.Unlock();
}


}  // namespace universal_logger
}  // namespace cogrob
