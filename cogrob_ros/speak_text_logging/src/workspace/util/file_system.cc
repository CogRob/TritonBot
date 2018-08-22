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

#include "util/file_system.h"

#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "third_party/glog.h"

using std::string;
using std::vector;

namespace util {

namespace internal {

void StringSplit(const string& input, char delim, vector<string>* result) {
  size_t begin = 0;
  size_t current = input.find(delim);
  while (current != string::npos) {
    result->push_back(input.substr(begin, current - begin));
    begin = current + 1;
    current = input.find(delim, begin);
  }
  if (begin <= input.length()) {
    result->push_back(input.substr(begin, input.length() - current));
  }
}

}  // namespace internal

namespace {
}  // namespace


Status MakeDirectories(const string& path_in) {
  string path = path_in;
  while (path.length() > 0 && path[path.length() - 1] == '/') {
    path.pop_back();
  }
  if (path.length() == 0) {
    return Status(util::error::INVALID_ARGUMENT, "Empty path string.");
  }
  if (path[0] != '/') {
    return Status(util::error::INVALID_ARGUMENT, "Abosulte path required.");
  }
  vector<string> path_components;
  internal::StringSplit(path, '/', &path_components);
  CHECK_GT(path_components.size(), 0);
  CHECK_EQ(path_components[0], "");

  string current_path;
  for (int i = 1; i < path_components.size(); ++i) {
    current_path += "/" + path_components[i];
    // Checks if current_path already exisits
    int mkdir_status =
        mkdir(current_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (mkdir_status) {
      // There is an error.
      if (errno == EEXIST) {
        // It is ok.
      } else {
        return Status(util::error::UNKNOWN, "Create " + current_path
            + " failed, errno = " + std::to_string(errno));
      }
    }
  }
  return Status::OK;
}

}  // namespace util
