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

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <iostream>
#include <string>
#include <stdexcept>

#include "jaguar4x4_comms/Communication.h"

int Communication::ipValidator(const std::string& ip)
{
  unsigned int n1, n2, n3, n4;
  if (sscanf(ip.c_str(), "%u.%u.%u.%u", &n1, &n2, &n3, &n4) != 4) {
    return -1;
  }

  if ((n1 != 0) && (n1 <= 255) && (n2 <= 255) && (n3 <= 255) && (n4 <= 255)) {
    return 0;
  }

  return -1;
}

void Communication::connect(const std::string& ip, uint16_t port)
{
  if (ipValidator(ip)) {
    throw std::runtime_error(ip + " not a valid IP address");
  }

  struct sockaddr_in _addr;
  memset(&_addr, 0, sizeof(_addr));
  _addr.sin_family = AF_INET;
  _addr.sin_port = htons(port);

  if (inet_aton(ip.c_str(), &_addr.sin_addr) == 0) {
    throw std::runtime_error(ip + " not a valid inet IP address");
  }

  //TCP
  if ((fd_ = socket(PF_INET, SOCK_STREAM,IPPROTO_TCP)) < 0) {
    throw std::runtime_error("Failed to create socket");
  }

  //TCP, setup connection here
  if (::connect(fd_, (struct sockaddr *) &_addr,sizeof(_addr)) < 0) {
    throw std::runtime_error("Failed to connect with robot. IP address "
                             + ip + " Port: " + std::to_string(port));
  }
}

void Communication::sendCommand(const std::string& cmd)
{
  bool done = false;
  const char* cmd_progress = cmd.c_str();
  size_t len_to_send = cmd.length();
  while (!done) {
    int retval = send(fd_, cmd_progress, len_to_send, 0 /* blocking */);
    if (retval < 0) {

      if (retval == EINTR) {
        continue;
      }
      throw std::runtime_error("Failed to send command: " + cmd
                               + std::string(::strerror(errno)));
    }

    len_to_send -= retval;
    if (len_to_send) {
      cmd_progress += retval;
      continue;
    }
    done = true;
  }
}

static uint64_t timespec_to_usecs(struct timespec *ts)
{
  return ts->tv_sec * 1e6 + ts->tv_nsec / 1e3;
}

std::string Communication::recvMessage(const std::string& boundary,
                                       int timeout_msec)
{
  std::string rcv_str;
  size_t pos;

  // First check if there is data left in the partial_buffer_ that we need
  // to deal with.
  pos = partial_buffer_.find(boundary);
  if (pos != std::string::npos) { // TODO... this substringing is
                                  // stupid expensive, be smarter
    rcv_str = partial_buffer_.substr(0, pos);
    partial_buffer_ = partial_buffer_.substr(pos + boundary.length());
    return rcv_str;
  }

  fd_set readfds;
  char rcv_chunk[1024];

  FD_ZERO(&readfds);
  FD_SET(fd_, &readfds);

  struct timespec start_ts;
  clock_gettime(CLOCK_MONOTONIC, &start_ts);
  uint64_t start_usecs = timespec_to_usecs(&start_ts);

  long usecs_left = timeout_msec * 1000;

  while (usecs_left > 0) {
    struct timespec current_ts;
    clock_gettime(CLOCK_MONOTONIC, &current_ts);

    uint64_t current_usecs = timespec_to_usecs(&current_ts);
    usecs_left -= (current_usecs - start_usecs);
    start_usecs = current_usecs;

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = usecs_left;

    int retval = select(fd_ + 1, &readfds, NULL, NULL, &tv);
    if (retval < 0) {
      if (retval == EINTR) {
        continue;
      }
      throw std::runtime_error("Failed to receive command: "
                               + std::string(::strerror(errno)));
    } else if (retval == 0) {
      // didn't receive a command, just return an empty string
      break;
    }

    if (!FD_ISSET(fd_, &readfds)) {
      throw std::runtime_error("Failed to receive command, bad fd");
    }

    int rcv_retval = recv(fd_, rcv_chunk, sizeof(rcv_chunk)-1, 0);
    if (rcv_retval < 0) {
      if (rcv_retval == EINTR) {
        continue;
      }
      // TODO: don't throw an error... store it internally, just retun
      // as a timeout, then on the next call, complete collection of data.
      throw std::runtime_error("Failed to receive chunk: "
                               + std::string(::strerror(errno)));
    }

    partial_buffer_.append(rcv_chunk, static_cast<size_t>(rcv_retval));

    // if we found a boundary, we're done
    pos = partial_buffer_.find(boundary);
    if (pos != std::string::npos) { // TODO... this substringing is
                                    // stupid expensive, be smarter
      rcv_str = partial_buffer_.substr(0, pos);
      partial_buffer_ = partial_buffer_.substr(pos + boundary.length());
      break;
    }
  }

  return rcv_str;
}
