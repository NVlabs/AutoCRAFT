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

PROJECT_NAMESPACE_START

class PlaceMgr {
public:
  PlaceMgr(CirDB& cir) : _cir(cir) {}
  ~PlaceMgr() {}

  bool solveSMT(const Int numThreads,
                const Real ar = 1,
                const Real fcUtil = 0.7,
                const Vector<String>& vRegNames = Vector<String>(),
                const Vector<Real>& vFcRegUtils = Vector<Real>(),
                const Vector<Real>& vBounds = Vector<Real>(),
                const Int iter = 1);
  bool solveCP(const Int numThreads, const Real ar = 1);
  bool solveMILP();
 
  void genEdgeCells();
  void genFillCells();

  void setSymCellOrients();
  void setRowCellOrients();

private:
  CirDB& _cir;
};

PROJECT_NAMESPACE_END
