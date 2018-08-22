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

#undef _GLIBCXX_USE_CXX11_ABI
#define _GLIBCXX_USE_CXX11_ABI 0
#include "snowboy_wrapper.h"

#include <cstring>
#include <memory>
#include <string>
#include "snowboy-detect.h"

namespace snowboy {

SnowboyWrapper::SnowboyWrapper(const char* const resource_filename,
    const char* const model_str) {
  snowboy_detect_ = new SnowboyDetect(std::string(resource_filename),
                                      std::string(model_str));
}

bool SnowboyWrapper::Reset() {
  return snowboy_detect_->Reset();
}

int SnowboyWrapper::RunDetectionBuf(
    const char* const buf, size_t buf_size, bool is_end) {
  const std::string data(buf, buf_size);
  return snowboy_detect_->RunDetection(data, is_end);
}

int SnowboyWrapper::RunDetection(
    const float* const data, const int array_length, bool is_end) {
  return snowboy_detect_->RunDetection(data, array_length, is_end);
}

int SnowboyWrapper::RunDetection(
    const int16_t* const data, const int array_length, bool is_end) {
  return snowboy_detect_->RunDetection(data, array_length, is_end);
}

int SnowboyWrapper::RunDetection(
    const int32_t* const data, const int array_length, bool is_end) {
  return snowboy_detect_->RunDetection(data, array_length, is_end);
}

void SnowboyWrapper::SetSensitivity(const char* const sensitivity_str) {
  snowboy_detect_->SetSensitivity(std::string(sensitivity_str));
}

void SnowboyWrapper::GetSensitivity(char* const dest, size_t buf_size) const {
  std::string result = snowboy_detect_->GetSensitivity();
  dest[buf_size - 1] = '\0';
  strncpy(dest, result.c_str(), buf_size - 1);
}

void SnowboyWrapper::SetAudioGain(const float audio_gain) {
  snowboy_detect_->SetAudioGain(audio_gain);
}

void SnowboyWrapper::UpdateModel() const {
  snowboy_detect_->UpdateModel();
}

int SnowboyWrapper::NumHotwords() const {
  return snowboy_detect_->NumHotwords();
}

void SnowboyWrapper::ApplyFrontend(const bool apply_frontend) {
  snowboy_detect_->ApplyFrontend(apply_frontend);
}

int SnowboyWrapper::SampleRate() const {
  return snowboy_detect_->SampleRate();
}

int SnowboyWrapper::NumChannels() const {
  return snowboy_detect_->NumChannels();
}

int SnowboyWrapper::BitsPerSample() const {
  return snowboy_detect_->BitsPerSample();
}

SnowboyWrapper::~SnowboyWrapper() {
  delete snowboy_detect_;
  snowboy_detect_ = nullptr;
}

}  // namespace snowboy
