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

#ifndef COGROB_UNIVERSAL_LOGGER_UNIVERSAL_LOGGER_INTERFACE_H_
#define COGROB_UNIVERSAL_LOGGER_UNIVERSAL_LOGGER_INTERFACE_H_

#include <string>

#include "cogrob/universal_logger/proto/archive_entry.pb.h"

#include "util/status.h"

namespace cogrob {
namespace universal_logger {

class UniversalLoggerFlusherInterface;

class UniversalLoggerInterface {
 public:
  // Writes an entry to the log file.
  virtual util::Status Log(const ArchiveEntry& entry) = 0;
  virtual util::Status Log(ArchiveEntry&& entry) = 0;

  // Flush the logger.
  virtual util::Status Flush() = 0;

  // Registers with a flusher that automatically flushes this logger
  // periodically.
  virtual util::Status RegisterWithFlusher(UniversalLoggerFlusherInterface*)
      = 0;

  virtual ~UniversalLoggerInterface() = default;
};

}  // namespace universal_logger
}  // namespace cogrob

#endif  // COGROB_UNIVERSAL_LOGGER_UNIVERSAL_LOGGER_INTERFACE_H_
