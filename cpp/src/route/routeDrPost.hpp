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

class PostRoute {
public:
  PostRoute(CirDB& cir)
    : _cir(cir)
  {}
  ~PostRoute() {}

  void patch();
  void patch(Region& reg);

private:
  CirDB& _cir;


  void initShapes(const Net& net, Vector<Vector<Box<Int>>>& vvBoxes);
  void checkShapes(const Vector<Vector<Box<Int>>>& vvBoxes, Vector<Vector<Box<Int>>>& vvPatches);
  void addPatches2Nets(Net& net, const Vector<Vector<Box<Int>>>& vvPatches);
};

PROJECT_NAMESPACE_END
