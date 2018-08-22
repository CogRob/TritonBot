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

#ifndef COGROB_UNIVERSAL_LOGGER_UNIVERSAL_LOGGER_H_
#define COGROB_UNIVERSAL_LOGGER_UNIVERSAL_LOGGER_H_
#include <unordered_set>
#include <string>
#include <mutex>

#include "cogrob/universal_logger/proto/archive_entry.pb.h"
#include "cogrob/universal_logger/proto/archive_header.pb.h"
#include "cogrob/universal_logger/universal_logger_interface.h"
#include "google/protobuf/io/zero_copy_stream.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/io/gzip_stream.h"
#include "third_party/gflags.h"

DECLARE_string(universal_logger_prefix);
DECLARE_int64(universal_logger_file_max_bytes);

namespace cogrob {
namespace universal_logger {

class UniversalLogger : public UniversalLoggerInterface {
 public:

  // Uses default options.
  UniversalLogger(const std::string& log_namespace);
  ~UniversalLogger();

  // Provides header information for UniversalLogger.
  UniversalLogger(
      const std::string& source_id, const std::string& log_namespace,
      const GeneralLogMetadata& metadata, bool rewrite_uuid,
      bool rewrite_timestamp, bool gzip_enabled);

  // Writes an entry to the log file.
  util::Status Log(const ArchiveEntry& entry) override;
  util::Status Log(ArchiveEntry&& entry) override;

  util::Status Flush() override;

  util::Status RegisterWithFlusher(UniversalLoggerFlusherInterface*) override;

 private:
  void CloseFile();
  void CreateNewFile();
  void InitUniversalLogger(
      const std::string& source_id, const std::string& log_namespace,
      const GeneralLogMetadata& metadata, bool rewrite_uuid,
      bool rewrite_timestamp, bool gzip_enabled);
  util::Status LogRaw(const ArchiveEntry& entry);

  static std::string GetDateStringFromPacked32(uint32_t packed32);
  static uint32_t GetDatePacked32();

  ArchiveHeader archive_header_;
  uint32_t file_date_packed32_ = 0;
  bool rewrite_uuid_ = false;
  bool rewrite_timestamp_ = true;
  int file_out_fd_ = 0;
  std::unique_ptr<google::protobuf::io::FileOutputStream> file_out_stream_;
  std::unique_ptr<google::protobuf::io::GzipOutputStream> gz_out_stream_;
  google::protobuf::io::ZeroCopyOutputStream* ostream_ = nullptr;
  std::mutex log_mutex_;
  bool gzip_enabled_ = true;
  std::string path_prefix_;
  std::string source_id_;
  std::string log_namespace_;
  int64_t file_max_bytes_ = 1024000;   // 100KiB, will be overridden by flags.
  std::unordered_set<UniversalLoggerFlusherInterface*> flushers_;
};

}  // namespace universal_logger
}  // namespace cogrob

#endif  // COGROB_UNIVERSAL_LOGGER_UNIVERSAL_LOGGER_H_
