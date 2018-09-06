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

#include <locale>
#include <sstream>
#include <string>

// Terminology from http://www.cplusplus.com/reference/string/string/compare/
bool startsWith(const std::string& compared,
                const std::string& comparing)
{
  if (compared.length() < comparing.length()) {
    return false;
  }
  return compared.compare(0, comparing.length(), comparing) == 0;
}

std::string dumpHex(const std::string& msg)
{
  static const char* const lut = "0123456789ABCDEF";
  std::string output;
  output.reserve(2 * msg.length());
  for (size_t i = 0; i < msg.length(); ++i) {
    output.push_back(lut[msg[i] >> 4]);
    output.push_back(lut[msg[i] & 15]);
  }
  return output;
}

double str_to_d(const std::string& in)
{
  std::stringstream ss;
  ss.imbue(std::locale::classic());
  ss << in;

  double out;
  ss >> out;

  if (ss.fail() || !ss.eof()) {
    throw std::runtime_error("failed str_to_d conversion");
  }

  return out;
}
