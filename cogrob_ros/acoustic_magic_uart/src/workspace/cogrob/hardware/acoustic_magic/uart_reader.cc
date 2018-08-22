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

#include "cogrob/hardware/acoustic_magic/uart_reader.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "util/simple_thread_safe_queue.h"
#include "third_party/glog.h"

namespace {

// False for failure, and true for success
// Adpoted from http://stackoverflow.com/questions/6947413
bool SetInterfaceAttribs(int fd, int speed) {
  struct termios tty;

  if (tcgetattr(fd, &tty) < 0) {
    LOG(ERROR) << "Error from tcsetattr:" << strerror(errno);
    return false;
  }

  cfsetospeed(&tty, (speed_t)speed);
  cfsetispeed(&tty, (speed_t)speed);
  tty.c_cflag |= (CLOCAL | CREAD);    // ignore modem controls
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;                 // 8-bit characters
  tty.c_cflag &= ~PARENB;             // no parity bit
  tty.c_cflag &= ~CSTOPB;             // only need 1 stop bit
  tty.c_cflag &= ~CRTSCTS;            // no hardware flowcontrol

  // Setup for non-canonical mode
  tty.c_iflag &= ~(
      IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_oflag &= ~OPOST;

  // Fetch bytes as they become available
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    LOG(ERROR) << "Error from tcsetattr:" << strerror(errno);
    return false;
  }
  return true;
}

}  // namespace

namespace cogrob {
namespace hardware {
namespace acoustic_magic {

UartReader::UartReader(const std::string& device_path, UartQueue* uart_queue) {
  thread_.reset(new std::thread([this, device_path, uart_queue] {
      ForeverLoopThread(device_path, uart_queue);
  }));
}

UartReader::~UartReader() {
  stop_.store(true);
  thread_->join();
}

void UartReader::ForeverLoopThread(
    const std::string& device_path, UartQueue* uart_queue) {
  // Open the port serial.
  int fd = open(device_path.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    LOG(FATAL) << "Open device " << device_path << " failed.";
  } else {
    LOG(INFO) << "Opened device " << device_path;
  }

  // Configure the prot
  if (!SetInterfaceAttribs(fd, B2400)) {
    LOG(FATAL) << "Setting baud rate to 2400 faild.";
  } else {
    LOG(INFO) << "Set baud rate to 2400.";
  }

  while (!stop_.load()) {
    // Read from the port
    uint8_t buf;
    size_t read_count = read(fd, &buf, 1);
    if (read_count != 1) {
      LOG(FATAL) << "Reading from serial port failed.";
      break;
    }
    uart_queue->push(buf);
  }
}

}  // namespace acoustic_magic
}  // namespace hardware
}  // namespace cogrob
