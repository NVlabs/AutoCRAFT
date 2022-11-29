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

#include <boost/polygon/polygon.hpp>
#include "polygon2box.hpp"
#include "polygon.hpp"
#include "ds/hash.hpp"

PROJECT_NAMESPACE_START

namespace geo {
  template<typename CoordType>
  inline boost::polygon::polygon_90_set_data<CoordType> box2NativePolygon(const Vector<Box<CoordType>>& vBoxes)
  {
    typedef boost::polygon::property_merge_90<CoordType, Int> PropertyMergeType; // use Int as property_type
    typedef boost::polygon::polygon_90_set_data<CoordType>  PolygonSetType;
    typedef Map<Set<Int>, PolygonSetType> PropertyMergeResultType;
    PropertyMergeType pm;
    for (size_t idx = 0; idx < vBoxes.size(); ++idx) {
      pm.insert(vBoxes[idx], 0); // Use 0 as property -> do not distinguish shapes
    }
    PropertyMergeResultType result;
    pm.merge(result);
    for (const auto& mapPair : result) {
      return mapPair.second;
    }
    assert(false);
    return (*result.begin()).second;
  }

  template<typename CoordType>
  void box2Polygon(const Vector<Box<CoordType>>& vBoxes, Vector<Polygon<CoordType>>& polygonVec)
  {
    if (vBoxes.empty()) {
      return;
    }
    const auto& polygonSet = box2NativePolygon(vBoxes);
    polygonSet.get_polygons(polygonVec);
  }

  // convert the boxes into nonoverlapping boxes
  template<typename CoordType>
    inline void boxes2Boxes(const Vector<Box<CoordType>>& vBoxes, Vector<Box<CoordType>>& results)
    {
      if (vBoxes.empty()) {
        return;
      }
      const auto& polygon = box2NativePolygon(vBoxes);
      polygon.get_rectangles(results);
    }

  // convert the boxes into nonoverlapping boxes
  template<typename CoordType>
  inline void boxes2BoxesEmplace(Vector<Box<CoordType>>& vBoxes)
  {
    if (vBoxes.empty()) {
      return;
    }
    const auto& polygon = box2NativePolygon(vBoxes);
    vBoxes.clear();
    polygon.get_rectangles(vBoxes);
  }


  // do difference on two vector of boxes
  template<typename CoordType>
  inline void boxesDiffAssign(Vector<Box<CoordType>>& lhs, const Vector<Box<CoordType>>& rhs)
  {
    typedef boost::polygon::polygon_90_set_data<CoordType> PolygonSetType;
    PolygonSetType lps, rps;
    for (const auto& rect : lhs) {
      lps.insert(rect);
    }
    for (const auto& rect : rhs) {
      rps.insert(rect);
    }
    boost::polygon::operators::operator-=(lps, rps);
    lhs.clear();
    lps.get_rectangles(lhs);
  }

  /// @brief do difference on two vector of boxes
  template<typename CoordType>
  inline void boxesAddAssign(Vector<Box<CoordType>>& lhs, const Vector<Box<CoordType>>& rhs)
  {
    typedef boost::polygon::polygon_90_set_data<CoordType> PolygonSetType;
    PolygonSetType lps, rps;
    for (const auto &rect : lhs)
    {
      lps.insert(rect);
    }
    for (const auto &rect : rhs)
    {
      rps.insert(rect);
    }
    boost::polygon::operators::operator+=(lps, rps);
    lhs.clear();
    lps.get_rectangles(lhs);
  }
};

PROJECT_NAMESPACE_END

