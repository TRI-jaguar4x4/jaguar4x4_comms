// Copyright 2018 Toyota Research Institute.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <string>

#include "AbstractCommunication.h"

class Communication final : public AbstractCommunication {
 public:
  void connect(const std::string& ip, uint16_t port) override;
  void sendCommand(const std::string& cmd) override;
  std::string recvMessage(const std::string& boundary, int timeout_msec)
    override;

 private:
  int ipValidator(const std::string& ip);
  int fd_;
  // TODO: restrict the partial_buffer so that it doesn't consume all of memory
  std::string partial_buffer_;
};
