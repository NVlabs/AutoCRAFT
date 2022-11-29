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

#include "dbLayer.hpp"

PROJECT_NAMESPACE_START

Int MetalLayer::eolIdx(const Int eolWidth) const
{
  //Int idx = 0;
  //for (size_t i = 0; i < _vEolWidths.size(); ++i) {
    //if (_vEolWidths.at(i) < eolWidth) {
      //idx = i;
    //}
    //else {
      //break;
    //}
  //}
  //return idx;
  const Int idx = std::upper_bound(_vEolWidths.begin(), _vEolWidths.end(), eolWidth) - _vEolWidths.begin();
  return idx >= static_cast<Int>(_vEolWidths.size()) ? _vEolWidths.size() - 1 : idx;
}

Int MetalLayer::prlSpacing(const Int width, const Int prl) const
{

  // find prl
  Int prlIdx = std::upper_bound(_vSpacingTablePrl.begin(), _vSpacingTablePrl.end(), prl) - _vSpacingTablePrl.begin() - 1;
  if (prlIdx >= static_cast<Int>(_vSpacingTablePrl.size())) {
    prlIdx = _vSpacingTablePrl.size() - 1;
  }
  assert(_vSpacingTablePrl.at(prlIdx) <= prl); 

  // find width
  Int wIdx = std::upper_bound(_vSpacingTableWidth.begin(), _vSpacingTableWidth.end(), width) - _vSpacingTableWidth.begin() - 1;
  if (wIdx >= static_cast<Int>(_vSpacingTableWidth.size())) {
    wIdx = _vSpacingTableWidth.size() - 1;
  } 
  assert(_vSpacingTableWidth.at(wIdx) <= width);

  return _spacingTable.at(wIdx, prlIdx);

}

Int MetalLayer::validWidthIdx(const Int w) const
{
  //Int idx = 0;
  //for (size_t i = 0; i < _vValidWidths.size(); ++i) {
    //if (_vValidWidths.at(i) >= w) {
      //idx = i;
      //break;
    //}
  //}
  //return idx;

  return std::lower_bound(_vValidWidths.begin(), _vValidWidths.end(), w) - _vValidWidths.begin();
}

Int MetalLayer::validLengthIdx(const Int l) const
{
  //Int idx = 0;
  //for (size_t i = 0; i < _vValidLengths.size(); ++i) {
    //if (_vValidLengths.at(i) >= l) {
      //idx = i;
      //break;
    //}
  //}
  //return idx;

  return std::lower_bound(_vValidLengths.begin(), _vValidLengths.end(), l) - _vValidLengths.begin();
}


PROJECT_NAMESPACE_END
