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

#include "cogrob/universal_logger/universal_logger.h"
#include "cogrob/universal_logger/universal_logger_flusher_interface.h"

#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <time.h>

#include <chrono>
#include <mutex>

#include "google/protobuf/util/delimited_message_util.h"
#include "third_party/glog.h"
#include "util/file_system.h"
#include "util/status_macros.h"
#include "util/timestamp.h"
#include "util/uuid.h"

using google::protobuf::io::FileOutputStream;
using google::protobuf::io::GzipOutputStream;
using google::protobuf::io::ZeroCopyOutputStream;
using google::protobuf::util::SerializeDelimitedToZeroCopyStream;
using util::Status;

DEFINE_string(universal_logger_prefix, "/home/cogrob_log",
              "Root directory for universal_logger, absolute path.");
DEFINE_int64(universal_logger_file_max_bytes, 1073741824,
             "Max size for a single universal_logger file.");

namespace {
std::string GetDefaultSourceId() {
  const int BUF_LEN = 1024;
  char hostname[BUF_LEN];
  hostname[BUF_LEN - 1] = '\0';
  gethostname(hostname, BUF_LEN - 1);
  return std::string(hostname);
}
}  // namespace

namespace cogrob {
namespace universal_logger {

UniversalLogger::UniversalLogger(
    const std::string& source_id, const std::string& log_namespace,
    const GeneralLogMetadata& metadata, bool rewrite_uuid,
    bool rewrite_timestamp, bool gzip_enabled) {
  std::lock_guard<std::mutex> lock(log_mutex_);
  InitUniversalLogger(source_id, log_namespace, metadata, rewrite_uuid,
                      rewrite_timestamp, gzip_enabled);
}

UniversalLogger::UniversalLogger(const std::string& log_namespace) {
  std::lock_guard<std::mutex> lock(log_mutex_);
  GeneralLogMetadata empty_metadata;
  InitUniversalLogger(
      GetDefaultSourceId(), log_namespace, empty_metadata, false, true, true);
}

UniversalLogger::~UniversalLogger() {
  std::lock_guard<std::mutex> lock(log_mutex_);
  for (auto* flusher : flushers_) {
    CHECK_OK(flusher->Unregister(this));
  }
  flushers_.clear();
}

util::Status UniversalLogger::RegisterWithFlusher(
    UniversalLoggerFlusherInterface* flusher) {
  std::lock_guard<std::mutex> lock(log_mutex_);
  RETURN_IF_ERROR(flusher->Register(this));
  flushers_.insert(flusher);
  return Status::OK;
}

void UniversalLogger::InitUniversalLogger(
    const std::string& source_id, const std::string& log_namespace,
    const GeneralLogMetadata& metadata, bool rewrite_uuid,
    bool rewrite_timestamp, bool gzip_enabled) {
  *(archive_header_.mutable_general_log()) = metadata;
  archive_header_.set_source_id(source_id);
  archive_header_.set_log_namespace(log_namespace);
  source_id_ = source_id;
  log_namespace_ = log_namespace;
  rewrite_uuid_ = rewrite_uuid;
  rewrite_timestamp_ = rewrite_timestamp;
  gzip_enabled_ = gzip_enabled;
  path_prefix_ = FLAGS_universal_logger_prefix;
  file_max_bytes_ = FLAGS_universal_logger_file_max_bytes;
  while (path_prefix_.length() &&
         path_prefix_[path_prefix_.length() - 1] == '/') {
    path_prefix_.pop_back();
  }
  while (log_namespace_.length() &&
         log_namespace_[log_namespace_.length() - 1] == '/') {
    log_namespace_.pop_back();
  }
  CreateNewFile();
}

void UniversalLogger::CloseFile() {
  // Here we assume the lock is held by the current thread.
  if (gz_out_stream_)  {
    CHECK(gz_out_stream_->Close());
    gz_out_stream_.reset();
  }
  if (file_out_stream_) {
    CHECK(file_out_stream_->Close());
    file_out_stream_.reset();
  }
  if (file_out_fd_) {
    close(file_out_fd_);
    ostream_ = nullptr;
  }
}

util::Status UniversalLogger::Log(const ArchiveEntry& entry) {
  std::lock_guard<std::mutex> lock(log_mutex_);
  if (rewrite_uuid_ || rewrite_timestamp_) {
    // Makes a copy and call LogRaw.
    ArchiveEntry to_write = entry;
    if (rewrite_uuid_) {
      util::NewUuid(to_write.mutable_uuid());
    }
    if (rewrite_timestamp_) {
      util::PopulateCurrentTimestamp(to_write.mutable_timestamp());
    }
    return LogRaw(to_write);
  }
  return LogRaw(entry);
}

util::Status UniversalLogger::Log(ArchiveEntry&& entry) {
  std::lock_guard<std::mutex> lock(log_mutex_);
  if (rewrite_uuid_) {
    util::NewUuid(entry.mutable_uuid());
  }
  if (rewrite_timestamp_) {
    util::PopulateCurrentTimestamp(entry.mutable_timestamp());
  }
  return LogRaw(entry);
}

util::Status UniversalLogger::Flush() {
  std::lock_guard<std::mutex> lock(log_mutex_);
  if (gz_out_stream_)  {
    CHECK(gz_out_stream_->Flush());
  }
  if (file_out_stream_) {
    CHECK(file_out_stream_->Flush());
  }
  return Status::OK;
}

util::Status UniversalLogger::LogRaw(const ArchiveEntry& entry) {
  if (file_out_stream_->ByteCount() >= file_max_bytes_) {
    CreateNewFile();
  } else if (GetDatePacked32() != file_date_packed32_) {
    CreateNewFile();
  }
  SerializeDelimitedToZeroCopyStream(entry, ostream_);
  return Status::OK;
}

std::string UniversalLogger::GetDateStringFromPacked32(uint32_t packed32) {
  uint32_t year = packed32 >> 9;
  uint32_t month = (packed32 >> 5) & 0x0F;
  uint32_t day = packed32 & 0x1F;
  return std::to_string(year) + "/"
         + (month < 10 ? "0": "") + std::to_string(month) + "/"
         + (day < 10 ? "0": "") + std::to_string(day);
}

uint32_t UniversalLogger::GetDatePacked32() {
  std::time_t now = std::chrono::system_clock::to_time_t(
      std::chrono::system_clock::now());
  struct tm time_parts {};
  localtime_r(&now, &time_parts);
  uint32_t result = ((time_parts.tm_year + 1900) << 9)
                    | ((time_parts.tm_mon + 1) << 5) | time_parts.tm_mday;
  return result;
}

void UniversalLogger::CreateNewFile() {
  // Here we assume the lock is held by the current thread.
  CloseFile();
  file_date_packed32_ = GetDatePacked32();
  std::string date_string = GetDateStringFromPacked32(file_date_packed32_);
  std::string file_dir = path_prefix_ + "/" + source_id_ + "/" + date_string
                         + "/" + log_namespace_;
  util::Status make_dir_status = util::MakeDirectories(file_dir);
  if (!make_dir_status.ok()) {
    LOG(FATAL) << make_dir_status;
  }
  std::string filename = file_dir + "/" +
      std::to_string(util::GetCurrentTimestampSec()) + ".pb";
  if (gzip_enabled_) {
    filename += ".gz";
  }
  LOG(INFO) << "Creating new UniversalLogger file " << filename.c_str();
  file_out_fd_ = open(
      filename.c_str(), O_WRONLY | O_CREAT,
      S_IWUSR | S_IRUSR | S_IRGRP | S_IWGRP | S_IROTH);
  CHECK_GE(file_out_fd_, 0);
  file_out_stream_.reset(new FileOutputStream(file_out_fd_));
  if (gzip_enabled_) {
    gz_out_stream_.reset(new GzipOutputStream(file_out_stream_.get()));
    ostream_ = gz_out_stream_.get();
  } else {
    ostream_ = file_out_stream_.get();
  }
  archive_header_.set_create_timestamp_sec(util::GetCurrentTimestampSec());
  SerializeDelimitedToZeroCopyStream(archive_header_, ostream_);
}

}  // namespace universal_logger
}  // namespace cogrob
