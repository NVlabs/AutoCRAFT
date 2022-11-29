/*
* SPDX-FileCopyrightText: Copyright (c) <year> NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#pragma once

#include "db/dbBasic.hpp"

PROJECT_NAMESPACE_START

class Net;

class NetPrioCstr {
  friend class Parser;

public:
  NetPrioCstr()
    : _idx(-1), _priority(0)
  {}

  const String&       name()              const { return _name; }
  Int                 idx()               const { return _idx; }

  Net&                net(const Int i)          { return *_vpNets.at(i); }
  const Net&          net(const Int i)    const { return *_vpNets.at(i); }
  Net*                pNet(const Int i)         { return _vpNets.at(i); }
  const Net*          pNet(const Int i)   const { return _vpNets.at(i); }
  Vector<Net*>&       vpNets()                  { return _vpNets; }
  const Vector<Net*>& vpNets()            const { return _vpNets; }
  Int                 numNets()           const { return _vpNets.size(); }

  Int                 priority()          const { return _priority; }

private:
  String       _name;
  Int          _idx;

  Vector<Net*> _vpNets;
  Int          _priority;
  
};

PROJECT_NAMESPACE_END
