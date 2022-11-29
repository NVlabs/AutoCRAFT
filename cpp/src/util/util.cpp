/*
* SPDX-FileCopyrightText: Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/
#include "util.hpp"
#include <sys/stat.h>

PROJECT_NAMESPACE_START

namespace util::fs {

bool existFile(const String& fileName)
{
  struct stat buf;
  return stat(fileName.c_str(), &buf) == 0;
}

String getDirName(const String& fileName)
{
  String            retStr = fileName;
  String::size_type pos    = retStr.rfind("/");
  if (pos != String::npos)
    retStr = retStr.substr(0, pos);
  return retStr;
}

String getFileName(const String& fileName)
{
  String            retStr = fileName;
  String::size_type pos    = retStr.rfind("/");
  if (pos != String::npos)
    retStr = retStr.substr(pos + 1);
  return retStr;
}

} // namespace util::fs

namespace util::str {

void split(const String& str, const String& delims, Vector<String>& tokens)
{
  tokens.clear();
  String s(str), token;
  s.erase(std::remove(s.begin(), s.end(), '\n'), s.end());
  size_t cur, prev = 0;
  cur = s.find_first_of(delims);
  while (cur != String::npos) {
    token = s.substr(prev, cur - prev);
    if (token != "") {
      tokens.emplace_back(token);
    }
    prev = cur + 1;
    cur  = s.find_first_of(delims, prev);
  }
  token = s.substr(prev, cur - prev);
  if (token != "") {
    tokens.emplace_back(token);
  }
}

} // namespace util::str

PROJECT_NAMESPACE_END
