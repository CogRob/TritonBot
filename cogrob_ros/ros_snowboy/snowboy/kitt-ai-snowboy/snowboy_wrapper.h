// Copyright (c) 2017, The Regents of the University of California
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

#ifndef SNOWBOY_WRAPPER_H
#define SNOWBOY_WRAPPER_H

#include <stdint.h>
#include <string.h>

namespace snowboy {

class SnowboyDetect;

class SnowboyWrapper {
 public:
  SnowboyWrapper(const char* const resource_filename,
                 const char* const model_str);
  bool Reset();
  int RunDetectionBuf(
      const char* const buf, size_t buf_size, bool is_end = false);
  int RunDetection(const float* const data,
                   const int array_length, bool is_end = false);
  int RunDetection(const int16_t* const data,
                   const int array_length, bool is_end = false);
  int RunDetection(const int32_t* const data,
                   const int array_length, bool is_end = false);
  void SetSensitivity(const char* const sensitivity_str);
  void GetSensitivity(char* const dest, size_t buf_size) const;
  void SetAudioGain(const float audio_gain);
  void UpdateModel() const;
  int NumHotwords() const;
  void ApplyFrontend(const bool apply_frontend);
  int SampleRate() const;
  int NumChannels() const;
  int BitsPerSample() const;
  ~SnowboyWrapper();
 private:
  SnowboyDetect* snowboy_detect_;
};

}  // namespace snowboy

#endif  // SNOWBOY_WRAPPER_H
