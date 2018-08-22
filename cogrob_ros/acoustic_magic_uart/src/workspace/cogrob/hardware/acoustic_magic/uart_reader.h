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

#ifndef COGROB_HARDWARE_ACOUSTIC_MAGIC_UART_READER_H_
#define COGROB_HARDWARE_ACOUSTIC_MAGIC_UART_READER_H_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "util/simple_thread_safe_queue.h"

namespace cogrob {
namespace hardware {
namespace acoustic_magic {

using UartQueue = util::SimpleThreadSafeQueue<uint8_t>;

class UartReader {
 public:
  // Automatically starts a new thread, and stops the thread on destruction.
  UartReader(
      const std::string& device_path,
      UartQueue* uart_queue);

  ~UartReader();
 private:
  void ForeverLoopThread(const std::string& device_path, UartQueue* uart_queue);
  std::atomic_bool stop_{false};
  std::unique_ptr<std::thread> thread_;
};

}  // namespace acoustic_magic
}  // namespace hardware
}  // namespace cogrob

#endif  // COGROB_HARDWARE_ACOUSTIC_MAGIC_UART_READER_H_
