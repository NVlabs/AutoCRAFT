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

#pragma once

#include "db/dbCir.hpp"
#include "routeDrMgr.hpp"
#include "drc/drcMgr.hpp"

PROJECT_NAMESPACE_START

class RouteMgr {
public:
  RouteMgr(CirDB& cir)
    : _cir(cir), _drc(cir)
  {
    
  }
  ~RouteMgr() {}
  
  ////////////////////////////////////
  //              PG                //
  ////////////////////////////////////
  void genPg();
  void genPgGrid(const Region& reg, const RouteGrid& rg, Net& net);
  void connectPgGrid(const Region& reg, Net& net);

  ////////////////////////////////////
  //              GR                //
  ////////////////////////////////////
  
  ////////////////////////////////////
  //              DR                //
  ////////////////////////////////////

private:
  CirDB& _cir;
  DrcMgr _drc;
};

PROJECT_NAMESPACE_END
