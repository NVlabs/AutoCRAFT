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

#include "dbNet.hpp"

PROJECT_NAMESPACE_START

void TopoTree::vRoutes(Vector<Segment3d<Int>>& vSegs) const
{
  for (auto it = _spatialSegs.begin(); it != _spatialSegs.end(); ++it) {
    const auto& b = it->first;
    const Point3d<Int>& p0 = b.min_corner();
    const Point3d<Int>& p1 = b.max_corner();
    vSegs.emplace_back(p0, p1);
#ifndef NDEBUG
    const auto& pp0 = _vPoints.at(_vSegs.at(it->second).first);
    const auto& pp1 = _vPoints.at(_vSegs.at(it->second).second);
    assert(pp0 == p0);
    assert(pp1 == p1);
#endif
  }
}

void TopoTree::vRoutes(Vector<Segment3d<Int>>& vSegs, Vector<SegData>& vSegData) const
{
  for (auto it = _spatialSegs.begin(); it != _spatialSegs.end(); ++it) {
    const auto& b = it->first;
    const Int segIdx = it->second;
    const Point3d<Int>& p0 = b.min_corner();
    const Point3d<Int>& p1 = b.max_corner();
    vSegs.emplace_back(p0, p1);
    vSegData.emplace_back(_vSegData.at(segIdx));
#ifndef NDEBUG
    const auto& pp0 = _vPoints.at(_vSegs.at(it->second).first);
    const auto& pp1 = _vPoints.at(_vSegs.at(it->second).second);
    assert(pp0 == p0);
    assert(pp1 == p1);
#endif
  }
}

void TopoTree::querySegs(const Point3d<Int>& minCorner,
                    const Point3d<Int>& maxCorner,
                    Vector<Segment3d<Int>>& vSegs,
                    Vector<SegData>& vSegData,
                    spatial3d::QueryType qt) const
{
  Vector<Int> vRes;
  _spatialSegs.query(minCorner, maxCorner, vRes, qt);
  for (const Int segIdx : vRes) {
    const auto& seg = _vSegs.at(segIdx);
    const auto& segData = _vSegData.at(segIdx);
    const Point3d<Int>& p0 = _vPoints.at(seg.first);
    const Point3d<Int>& p1 = _vPoints.at(seg.second);
    vSegs.emplace_back(p0, p1);
    vSegData.emplace_back(segData);
  }
}

void TopoTree::vAdjSegs(const Segment3d<Int>& seg, Vector<Segment3d<Int>>& vAdjs) const
{
  assert(seg.b90());
  Vector<Int> vResults;
  _spatialSegs.query(seg.min_corner(), seg.max_corner(), vResults);
  for (const Int segIdx : vResults) {
    const Point3d<Int>& p0 = _vPoints[_vSegs[segIdx].first];
    const Point3d<Int>& p1 = _vPoints[_vSegs[segIdx].second];
    vAdjs.emplace_back(p0, p1);
  }
}

void TopoTree::buildSpatialPins()
{
  _spatialPins.clear();
  for (size_t i = 0; i < _vpPins.size(); ++i) {
    const Pin& pin = *_vpPins.at(i);
    Int j, layerIdx;
    const Box<Int>* pBox;
    Pin_ForEachLayerIdx(pin, layerIdx) {
      Pin_ForEachLayerBox(pin, layerIdx, pBox, j) {
        const Point3d<Int> minCorner(pBox->xl(), pBox->yl(), layerIdx);
        const Point3d<Int> maxCorner(pBox->xh(), pBox->yh(), layerIdx);
        _spatialPins.insert(minCorner, maxCorner, i);
      }
    }
  }
}

void TopoTree::addRoute(const Point3d<Int>& u, const Point3d<Int>& v, const SegData& segData)
{

  assert((u.x() != v.x()) + (u.y() != v.y()) + (u.z() != v.z()) == 1);

  const Point3d<Int> P0(std::min(u.x(), v.x()),
                        std::min(u.y(), v.y()),
                        std::min(u.z(), v.z()));
  const Point3d<Int> P1(std::max(u.x(), v.x()),
                        std::max(u.y(), v.y()),
                        std::max(u.z(), v.z()));

  auto getDim = [] (const Point3d<Int>& p0, const Point3d<Int>& p1) -> Byte {
    if (p0.x() != p1.x()) return 0;
    if (p0.y() != p1.y()) return 1;
    if (p0.z() != p1.z()) return 2;
    assert(false);
  };

  Vector<Int> vResults;
  _spatialSegs.query(P0, P1, vResults);
  FlatHashMap<Pair<Int, Int>, Int> removedSegs;
  for (Int dim = 0; dim < 3; ++dim) {
    if (P0.val(dim) == P1.val(dim))
      continue;
    Int low = P0.val(dim);
    Int high = P1.val(dim);
    Vector<Int> vOrthoResults;
    for (size_t i = 0; i < vResults.size(); ++i) {
      const Int segIdx = vResults.at(i);
      const Int p0Idx = _vSegs.at(segIdx).first;
      const Int p1Idx = _vSegs.at(segIdx).second;
      const auto& p0 = _vPoints.at(p0Idx);
      const auto& p1 = _vPoints.at(p1Idx);
      assert(p0.x() <= p1.x() and p0.y() <= p1.y() and p0.z() <= p1.z());
      if (dim == getDim(p0, p1)) {
        low = std::min({low, p0.val(dim), p1.val(dim)});
        high = std::max({high, p0.val(dim), p1.val(dim)});
        removedSegs.emplace(std::make_pair(p0Idx, p1Idx), segIdx);
        removeSeg(p0, p1, segIdx);
      }
      else {
        vOrthoResults.emplace_back(segIdx);
      }
    }
    Vector<Int> vBreakPts(1, low);
    for (size_t i = 0; i < vOrthoResults.size(); ++i) {
      const Int segIdx = vOrthoResults.at(i);
      const Int p0Idx = _vSegs.at(segIdx).first;
      const Int p1Idx = _vSegs.at(segIdx).second;
      const auto& p0 = _vPoints.at(p0Idx);
      const auto& p1 = _vPoints.at(p1Idx);
      assert(p0.x() <= p1.x() and p0.y() <= p1.y() and p0.z() <= p1.z());
      assert(p0.val(dim) == p1.val(dim));
      for (Int o_dim = 0; o_dim < 3; ++o_dim) {
        if (p0.val(o_dim) == p1.val(o_dim))
          continue;
        assert(o_dim != dim);
        assert(P0.val(o_dim) == P1.val(o_dim));
        const Int v0 = p0.val(o_dim);
        const Int v1 = p1.val(o_dim);
        if (v0 != P0.val(o_dim) and v1 != P0.val(o_dim)) {
          vBreakPts.emplace_back(p0.val(dim));
          assert(v0 < P0.val(o_dim) and v1 > P0.val(o_dim));

          const SegData sd = _vSegData.at(segIdx);
          removeSeg(p0, p1, segIdx);
          Point3d<Int> newPt(p0);
          newPt.val(o_dim) = P0.val(o_dim);

          const Int newPtIdx = addPoint(newPt);
          insertSeg(p0Idx, newPtIdx, sd);
          insertSeg(newPtIdx, p1Idx, sd);
        //std::cerr << newPt <<  p1 << " "<< segData.pVia << " " << util::enumUtil::val2Str(Orient2dEStr, segData.orient) << " " << segData.width << " "  << segData.ext <<std::endl;
        }
        else {
          //if (v0 == P0.val(o_dim)) {
            //vBreakPts.emplace_back(p0.val(dim));
          //}
          //else {
            //vBreakPts.emplace_back(p1.val(dim));
          //}
          assert(p0.val(dim) == p1.val(dim));
          vBreakPts.emplace_back(p0.val(dim));
          assert(p0.val(dim) >= low and p0.val(dim) <= high);
        }
        break;
      }
    }
    
    vBreakPts.emplace_back(high);
    if (dim == 2) {
      auto mnmx = std::minmax_element(vBreakPts.begin(), vBreakPts.end());
      const Int zl = *mnmx.first;
      const Int zh = *mnmx.second;
      assert((zh - zl) % 2 == 0);
      for (Int z = zl; z + 2 <= zh; z += 2) {
        Point3d<Int> newPt0(P0);
        Point3d<Int> newPt1(P1);
        newPt0.val(dim) = z;
        newPt1.val(dim) = z + 2;
        const Int newPt0Idx = addPoint(newPt0);
        const Int newPt1Idx = addPoint(newPt1);
        if (removedSegs.find({newPt0Idx, newPt1Idx}) != removedSegs.end()) {
          insertSeg(newPt0Idx, newPt1Idx, _vSegData.at(removedSegs.at({newPt0Idx, newPt1Idx})));
        }
        else {
          insertSeg(newPt0Idx, newPt1Idx, segData);
        }
      }
    }
    else {
      //Vector<Int> vTouchedPinIds;
      //_spatialPins.query(P0, P1, vTouchedPinIds);
      //for (const Int pinIdx : vTouchedPinIds) {
        //const Pin& pin = this->pin(pinIdx);
        //if (dim == 0) {
          //assert(P0.y() == P1.y());
          //assert(pin.cell().loc().y() == P0.y());
          //vBreakPts.emplace_back(pin.cell().loc().x());
          //assert(pin.cell().loc().x() >= low);
          //assert(pin.cell().loc().x() <= high);
        //}
        //else {
          //assert(dim == 1);
          //assert(P0.x() == P1.x());
          //assert(pin.cell().loc().x() == P0.x());
          //vBreakPts.emplace_back(pin.cell().loc().y());
          //assert(pin.cell().loc().y() >= low);
          //assert(pin.cell().loc().y() <= high);
        //}
      //}
      Vector<Int> vPinIds;
      _spatialPins.query(P0, P1, vPinIds);
      for (const Int pinIdx : vPinIds) {
        for (const auto& acs : _vpPins.at(pinIdx)->vAcs()) {
          if (dim == 0) {
            if (acs.z() == P0.z() and acs.y() == P0.y()) {
              if (low <= acs.x() and acs.x() <= high) {
                vBreakPts.emplace_back(acs.x());
              }
            }
          }
          else if (dim == 1) {
            if (acs.z() == P0.z() and acs.x() == P0.x()) {
              if (low <= acs.y() and acs.y() <= high) {
                vBreakPts.emplace_back(acs.y());
              }
            }
          }
        }
      }
      std::sort(vBreakPts.begin(), vBreakPts.end());
      assert(vBreakPts[0] >= low and vBreakPts.back() <= high);
      const Int breakSize = std::unique(vBreakPts.begin(), vBreakPts.end()) - vBreakPts.begin();
      for (Int i = 1; i < breakSize; ++i) {
        Point3d<Int> newPt0(P0);
        Point3d<Int> newPt1(P1);
        newPt0.val(dim) = vBreakPts[i - 1];
        newPt1.val(dim) = vBreakPts[i];
        const Int newPt0Idx = addPoint(newPt0);
        const Int newPt1Idx = addPoint(newPt1);
        insertSeg(newPt0Idx, newPt1Idx, segData);
      }
    }
    break;
  }
  
}

void TopoTree::addRoute(const Segment3d<Int>& seg, const SegData& segData)
{
  assert(seg.b90());
  addRoute(seg.p0(), seg.p1(), segData);
}
  
void TopoTree::addRoute(const Point3d<Int>& u, const Point3d<Int>& v, const SegData& segData, Vector<Tuple<Segment3d<Int>, SegData, bool>>& vAdjustedSegs)
{

  assert((u.x() != v.x()) + (u.y() != v.y()) + (u.z() != v.z()) == 1);

  const Point3d<Int> P0(std::min(u.x(), v.x()),
                        std::min(u.y(), v.y()),
                        std::min(u.z(), v.z()));
  const Point3d<Int> P1(std::max(u.x(), v.x()),
                        std::max(u.y(), v.y()),
                        std::max(u.z(), v.z()));

  auto getDim = [] (const Point3d<Int>& p0, const Point3d<Int>& p1) -> Byte {
    if (p0.x() != p1.x()) return 0;
    if (p0.y() != p1.y()) return 1;
    if (p0.z() != p1.z()) return 2;
    assert(false);
  };

  Vector<Int> vResults;
  _spatialSegs.query(P0, P1, vResults);
  FlatHashMap<Pair<Int, Int>, Int> removedSegs;
  for (Int dim = 0; dim < 3; ++dim) {
    if (P0.val(dim) == P1.val(dim))
      continue;
    Int low = P0.val(dim);
    Int high = P1.val(dim);
    Vector<Int> vOrthoResults;
    for (size_t i = 0; i < vResults.size(); ++i) {
      const Int segIdx = vResults.at(i);
      const Int p0Idx = _vSegs.at(segIdx).first;
      const Int p1Idx = _vSegs.at(segIdx).second;
      const auto& p0 = _vPoints.at(p0Idx);
      const auto& p1 = _vPoints.at(p1Idx);
      assert(p0.x() <= p1.x() and p0.y() <= p1.y() and p0.z() <= p1.z());
      if (dim == getDim(p0, p1)) {
        low = std::min({low, p0.val(dim), p1.val(dim)});
        high = std::max({high, p0.val(dim), p1.val(dim)});
        vAdjustedSegs.emplace_back(Segment3d<Int>(p0, p1), _vSegData.at(segIdx), false);
        removedSegs.emplace(std::make_pair(p0Idx, p1Idx), segIdx);
        removeSeg(p0, p1, segIdx);
      }
      else {
        vOrthoResults.emplace_back(segIdx);
      }
    }
    Vector<Int> vBreakPts(1, low);
    for (size_t i = 0; i < vOrthoResults.size(); ++i) {
      const Int segIdx = vOrthoResults.at(i);
      const Int p0Idx = _vSegs.at(segIdx).first;
      const Int p1Idx = _vSegs.at(segIdx).second;
      const auto& p0 = _vPoints.at(p0Idx);
      const auto& p1 = _vPoints.at(p1Idx);
      assert(p0.x() <= p1.x() and p0.y() <= p1.y() and p0.z() <= p1.z());
      assert(p0.val(dim) == p1.val(dim));
      for (Int o_dim = 0; o_dim < 3; ++o_dim) {
        if (p0.val(o_dim) == p1.val(o_dim))
          continue;
        assert(o_dim != dim);
        assert(P0.val(o_dim) == P1.val(o_dim));
        const Int v0 = p0.val(o_dim);
        const Int v1 = p1.val(o_dim);
        if (v0 != P0.val(o_dim) and v1 != P0.val(o_dim)) {
          const SegData sd = _vSegData.at(segIdx);

          vBreakPts.emplace_back(p0.val(dim));
          assert(v0 < P0.val(o_dim) and v1 > P0.val(o_dim));

          vAdjustedSegs.emplace_back(Segment3d<Int>(p0, p1), sd, false);
          removeSeg(p0, p1, segIdx);

          Point3d<Int> newPt(p0);
          newPt.val(o_dim) = P0.val(o_dim);
          const Int newPtIdx = addPoint(newPt); // p0, p1 invalidated after here
          vAdjustedSegs.emplace_back(Segment3d<Int>(_vPoints.at(p0Idx), newPt), sd, true);
          vAdjustedSegs.emplace_back(Segment3d<Int>(newPt, _vPoints.at(p1Idx)), sd, true);
          insertSeg(p0Idx, newPtIdx, sd);
          insertSeg(newPtIdx, p1Idx, sd);
        }
        else {
          //if (v0 == P0.val(o_dim)) {
            //vBreakPts.emplace_back(p0.val(dim));
          //}
          //else {
            //vBreakPts.emplace_back(p1.val(dim));
          //}
          assert(p0.val(dim) == p1.val(dim));
          vBreakPts.emplace_back(p0.val(dim));
          assert(p0.val(dim) >= low and p0.val(dim) <= high);
        }
        break;
      }
    }
    vBreakPts.emplace_back(high);
    if (dim == 2) {
      auto mnmx = std::minmax_element(vBreakPts.begin(), vBreakPts.end());
      const Int zl = *mnmx.first;
      const Int zh = *mnmx.second;
      assert((zh - zl) % 2 == 0);
      for (Int z = zl; z + 2 <= zh; z += 2) {
        Point3d<Int> newPt0(P0);
        Point3d<Int> newPt1(P1);
        newPt0.val(dim) = z;
        newPt1.val(dim) = z + 2;
        const Int newPt0Idx = addPoint(newPt0);
        const Int newPt1Idx = addPoint(newPt1);
        if (removedSegs.find({newPt0Idx, newPt1Idx}) != removedSegs.end()) {
          const SegData& sd = _vSegData.at(removedSegs.at({newPt0Idx, newPt1Idx}));
          vAdjustedSegs.emplace_back(Segment3d<Int>(newPt0, newPt1), sd, true);
          insertSeg(newPt0Idx, newPt1Idx, sd);
        }
        else {
          vAdjustedSegs.emplace_back(Segment3d<Int>(newPt0, newPt1), segData, true);
          insertSeg(newPt0Idx, newPt1Idx, segData);
        }
      }
    }
    else {
      //Vector<Int> vTouchedPinIds;
      //_spatialPins.query(P0, P1, vTouchedPinIds);
      //for (const Int pinIdx : vTouchedPinIds) {
        //const Pin& pin = this->pin(pinIdx);
        //if (dim == 0) {
          //assert(P0.y() == P1.y());
          //assert(pin.cell().loc().y() == P0.y());
          //vBreakPts.emplace_back(pin.cell().loc().x());
          //assert(pin.cell().loc().x() >= low);
          //assert(pin.cell().loc().x() <= high);
        //}
        //else {
          //assert(dim == 1);
          //assert(P0.x() == P1.x());
          //assert(pin.cell().loc().x() == P0.x());
          //vBreakPts.emplace_back(pin.cell().loc().y());
          //assert(pin.cell().loc().y() >= low);
          //assert(pin.cell().loc().y() <= high);
        //}
      //}
      Vector<Int> vPinIds;
      _spatialPins.query(P0, P1, vPinIds);
      for (const Int pinIdx : vPinIds) {
        for (const auto& acs : _vpPins.at(pinIdx)->vAcs()) {
          if (dim == 0) {
            if (acs.z() == P0.z() and acs.y() == P0.y()) {
              if (low <= acs.x() and acs.x() <= high) {
                vBreakPts.emplace_back(acs.x());
              }
            }
          }
          else if (dim == 1) {
            if (acs.z() == P0.z() and acs.x() == P0.x()) {
              if (low <= acs.y() and acs.y() <= high) {
                vBreakPts.emplace_back(acs.y());
              }
            }
          }
        }
      }
      std::sort(vBreakPts.begin(), vBreakPts.end());
      assert(vBreakPts[0] >= low and vBreakPts.back() <= high);
      const Int breakSize = std::unique(vBreakPts.begin(), vBreakPts.end()) - vBreakPts.begin();
      for (Int i = 1; i < breakSize; ++i) {
        Point3d<Int> newPt0(P0);
        Point3d<Int> newPt1(P1);
        newPt0.val(dim) = vBreakPts[i - 1];
        newPt1.val(dim) = vBreakPts[i];
        const Int newPt0Idx = addPoint(newPt0);
        const Int newPt1Idx = addPoint(newPt1);
        vAdjustedSegs.emplace_back(Segment3d<Int>(newPt0, newPt1), segData, true);
        insertSeg(newPt0Idx, newPt1Idx, segData);
      }
    }
    break;
  }

}

void TopoTree::addRoute(const Segment3d<Int>& seg, const SegData& segData, Vector<Tuple<Segment3d<Int>, SegData, bool>>& vAdjustedSegs)
{
  assert(seg.b90());
  addRoute(seg.p0(), seg.p1(), segData, vAdjustedSegs);
}

bool TopoTree::removeRoute(const Point3d<Int>& u, const Point3d<Int>& v, const bool bMerge)
{
  assert((u.x() != v.x()) + (u.y() != v.y()) + (u.z() != v.z()) == 1);

  const Point3d<Int> min_corner(std::min(u.x(), v.x()),
                                std::min(u.y(), v.y()),
                                std::min(u.z(), v.z()));
  const Point3d<Int> max_corner(std::max(u.x(), v.x()),
                                std::max(u.y(), v.y()),
                                std::max(u.z(), v.z()));
  
  const bool b = removeSeg(min_corner, max_corner, -2);
  assert(b);
  if (!b) {
    return false;
  }
  if (!bMerge) {
    return true;
  }


  auto getDim = [] (const Point3d<Int>& p0, const Point3d<Int>& p1) -> Byte {
    if (p0.x() != p1.x()) return 0;
    if (p0.y() != p1.y()) return 1;
    if (p0.z() != p1.z()) return 2;
    assert(false);
  };

  
  auto mergeSegs = [&] (const Point3d<Int>& pt) {
    Vector<Int> vPinIds;
    _spatialPins.query(pt, pt, vPinIds);
    assert(vPinIds.size() <= 1);
    for (const Int pinIdx : vPinIds) {
      for (const auto& acs : _vpPins.at(pinIdx)->vAcs()) {
        if (acs == pt) {
          return;
        }
      }
    }

    Vector<Int> vTouchedSegs;
    _spatialSegs.query(pt, pt, vTouchedSegs);

    // catagorized per dim
    Array<Vector<Int>, 3> arr;
    for (const Int segIdx : vTouchedSegs) {
      const auto& s = _vSegs.at(segIdx);
      const auto& p0 = _vPoints.at(s.first);
      const auto& p1 = _vPoints.at(s.second);
      const Int dim = getDim(p0, p1);
      arr.at(dim).emplace_back(segIdx);
    }
    const Int nonEmptyDim = !arr.at(0).empty() + !arr.at(1).empty() + !arr.at(2).empty();
    if (nonEmptyDim > 1) {
      return;
    }
    
    //for (Int dim = 0; dim < 3; ++dim) {
    for (Int dim = 0; dim < 2; ++dim) { // skip vias merging
      const auto& vSegs = arr.at(dim);
      assert(vSegs.size() <= 2);
      if (vSegs.size() < 2) {
        continue; 
      }
      if (!vSegs.empty()) {
        const auto& sd = _vSegData.at(vSegs.at(0));
#ifndef NDEBUG
        for (size_t i = 1; i < vSegs.size(); ++i) {
          const Int s0 = vSegs.at(i - 1);
          const Int s1 = vSegs.at(i);
          assert(_vSegData.at(s0) == _vSegData.at(s1));
        }
#endif

        Int newP0Idx = -1, newP1Idx = -1;

        Int low = MAX_INT, high = MIN_INT;
        for (const Int segIdx : vSegs) {
          const Int p0Idx = _vSegs.at(segIdx).first;
          const Int p1Idx = _vSegs.at(segIdx).second;
          const auto& p0 = _vPoints.at(p0Idx);
          const auto& p1 = _vPoints.at(p1Idx);
          const Int l = p0.val(dim), h = p1.val(dim);
          assert(l < h);
          if (l < low) {
            low = l;
            newP0Idx = p0Idx;
          }
          if (h > high) {
            high = h;
            newP1Idx = p1Idx;
          }
          removeSeg(p0, p1, segIdx);
        }
        assert(newP0Idx >= 0 and newP1Idx >= 0);
        insertSeg(newP0Idx, newP1Idx, sd);
      }
    }

  };

  mergeSegs(min_corner);
  mergeSegs(max_corner);
  return true;
}

bool TopoTree::removeRoute(const Segment3d<Int>& seg, const bool bMerge)
{
  assert(seg.b90());
  return removeRoute(seg.p0(), seg.p1(), bMerge);
}

bool TopoTree::removeRoute(const Point3d<Int>& u, const Point3d<Int>& v, Vector<Tuple<Segment3d<Int>, SegData, bool>>& vAdjustedSegs, const bool bMerge)
{
  assert((u.x() != v.x()) + (u.y() != v.y()) + (u.z() != v.z()) == 1);

  const Point3d<Int> min_corner(std::min(u.x(), v.x()),
                                std::min(u.y(), v.y()),
                                std::min(u.z(), v.z()));
  const Point3d<Int> max_corner(std::max(u.x(), v.x()),
                                std::max(u.y(), v.y()),
                                std::max(u.z(), v.z()));
  
  Int tarSegIdx;
  Vector<Int> vTouchedSegs;
  _spatialSegs.query(min_corner, max_corner, vTouchedSegs);
  for (const Int segIdx : vTouchedSegs) {
    const auto& p0 = _vPoints.at(_vSegs.at(segIdx).first);   
    const auto& p1 = _vPoints.at(_vSegs.at(segIdx).second); 
    if (p0 == min_corner and p1 == max_corner) {
      tarSegIdx = segIdx;
      break;
    }
  }

  vAdjustedSegs.emplace_back(Segment3d<Int>(min_corner, max_corner), _vSegData.at(tarSegIdx), false); // segdata doesn't matter here
  const bool b = removeSeg(min_corner, max_corner, tarSegIdx);
  assert(b);
  if (!b) {
    return false;
  }
  if (!bMerge) {
    return true;
  }



  auto getDim = [] (const Point3d<Int>& p0, const Point3d<Int>& p1) -> Byte {
    if (p0.x() != p1.x()) return 0;
    if (p0.y() != p1.y()) return 1;
    if (p0.z() != p1.z()) return 2;
    assert(false);
  };

  auto mergeSegs = [&] (const Point3d<Int>& pt) {
    Vector<Int> vPinIds;
    _spatialPins.query(pt, pt, vPinIds);
    assert(vPinIds.size() <= 1);
    for (const Int pinIdx : vPinIds) {
      for (const auto& acs : _vpPins.at(pinIdx)->vAcs()) {
        if (acs == pt) {
          return;
        }
      }
    }

    Vector<Int> vTouchedSegs;
    _spatialSegs.query(pt, pt, vTouchedSegs);
    
    // catagorized per dim
    Array<Vector<Int>, 3> arr;
    for (const Int segIdx : vTouchedSegs) {
      const auto& p0 = _vPoints.at(_vSegs.at(segIdx).first);
      const auto& p1 = _vPoints.at(_vSegs.at(segIdx).second);
      const Int dim = getDim(p0, p1);
      arr.at(dim).emplace_back(segIdx);
    }
    const Int nonEmptyDim = !arr.at(0).empty() + !arr.at(1).empty() + !arr.at(2).empty();
    if (nonEmptyDim > 1) {
      return;
    }
    
    //for (Int dim = 0; dim < 3; ++dim) {
    for (Int dim = 0; dim < 2; ++dim) { // skip vias merging
      const auto& vSegs = arr.at(dim);
      assert(vSegs.size() <= 2);
      if (vSegs.size() < 2) {
        continue; 
      }
      if (!vSegs.empty()) {
        const auto& sd = _vSegData.at(vSegs.at(0));
#ifndef NDEBUG
        for (size_t i = 1; i < vSegs.size(); ++i) {
          const Int s0 = vSegs.at(i - 1);
          const Int s1 = vSegs.at(i);
          const auto& sd0 = _vSegData.at(s0);
          const auto& sd1 = _vSegData.at(s1);
          //if (sd0 != sd1) {
            //this->showComps();

            //std::cerr << u << v << std::endl;
            //std::cerr << "dim: " << dim << std::endl;
            //const auto& p00 = _vPoints.at(_vSegs.at(s0).first);
            //const auto& p01 = _vPoints.at(_vSegs.at(s0).second);
            //const auto& p10 = _vPoints.at(_vSegs.at(s1).first);
            //const auto& p11 = _vPoints.at(_vSegs.at(s1).second);
             
            //std::cerr << "s0: " << p00 << p01 << " " << (sd0.pVia ? sd0.pVia->name() : "null") << " " << util::enumUtil::val2Str(Orient2dEStr, sd0.orient) << " " << sd0.width << " " << sd0.ext << std::endl;
            //std::cerr << "s1: " << p10 << p11 << " " << (sd1.pVia ? sd1.pVia->name() : "null") << " " << util::enumUtil::val2Str(Orient2dEStr, sd1.orient) << " " << sd1.width << " " << sd1.ext << std::endl;
          //}
          assert(sd0 == sd1);
        }
#endif

        Int newP0Idx = -1, newP1Idx = -1;

        Int low = MAX_INT, high = MIN_INT;
        for (const Int segIdx : vSegs) {
          const Int p0Idx = _vSegs.at(segIdx).first;
          const Int p1Idx = _vSegs.at(segIdx).second;
          const auto& p0 = _vPoints.at(p0Idx);
          const auto& p1 = _vPoints.at(p1Idx);
          const Int l = p0.val(dim), h = p1.val(dim);
          assert(l < h);
          if (l < low) {
            low = l;
            newP0Idx = p0Idx;
          }
          if (h > high) {
            high = h;
            newP1Idx = p1Idx;
          }
          vAdjustedSegs.emplace_back(Segment3d<Int>(p0, p1), _vSegData.at(segIdx), false);
          removeSeg(p0, p1, segIdx);
        }
        assert(newP0Idx >= 0 and newP1Idx >= 0);
        vAdjustedSegs.emplace_back(Segment3d<Int>(_vPoints.at(newP0Idx), _vPoints.at(newP1Idx)), sd, true);
        insertSeg(newP0Idx, newP1Idx, sd);
      }
    }

  };

  mergeSegs(min_corner);
  mergeSegs(max_corner);
  return true;

}

bool TopoTree::removeRoute(const Segment3d<Int>& seg, Vector<Tuple<Segment3d<Int>, SegData, bool>>& vAdjustedSegs, const bool bMerge)
{
  assert(seg.b90());
  return removeRoute(seg.p0(), seg.p1(), vAdjustedSegs, bMerge);
}

void TopoTree::removeAdjSegs(const Segment3d<Int>& seg)
{
  assert(seg.b90());
  Vector<Int> vResults;
  _spatialSegs.query(seg.min_corner(), seg.max_corner(), vResults);
  for (const Int segIdx : vResults) {
    const auto& p0 = _vPoints.at(_vSegs.at(segIdx).first);
    const auto& p1 = _vPoints.at(_vSegs.at(segIdx).second);
    removeSeg(p0, p1, segIdx);
  }
}

void TopoTree::refineDangling(const bool removeFloating, const bool removeRedundant)
{
  Vector<Tuple<Segment3d<int>, SegData, bool>> vAdjustedSegs;
  refineDangling(vAdjustedSegs, removeFloating, removeRedundant);
}

void TopoTree::refineDangling(Vector<Tuple<Segment3d<Int>, SegData, bool>>& vAdjustedSegs, const bool removeFloating, const bool removeRedundant)
{
  Vector<Int> par(_vPoints.size(), -1);
  Vector<char> isTerminal(_vPoints.size(), 0);
  Vector<char> deg(_vPoints.size(), 0);
  Vector<char> vis(_vPoints.size(), 0);
  assert(_vvAdjPtIds.size() == _vPoints.size());
  for (size_t i = 0; i < _vPoints.size(); ++i) {
    deg[i] = _vvAdjPtIds[i].size();
    isTerminal[i] = bTouchPin(_vPoints[i]);
  }

  for (size_t i = 0; i < _vPoints.size(); ++i) {
    if (!vis[i] and isTerminal[i]) {
      Queue<Int> q;
      dfsDeg(i, i, par, vis, deg, q);
      while (!q.empty()) {
        const Int u = q.front();
        q.pop();
        if (!isTerminal[u]) {
          if (par[u] != -1) {
            const Int p0Idx = u;
            const Int p1Idx = par[u];
            auto& p0 = _vPoints.at(p0Idx);
            auto& p1 = _vPoints.at(p1Idx);
            const Segment3d<Int> seg(p0, p1);
            assert(seg.b90());
            
            //for (Int dim = 0; dim < 3; ++dim) {
              //if (p0.val(dim) != p1.val(dim)) {
                //if (p0 < p1) {
                  //removeSeg(p0, p1, -1);
                  //vRemovedSegs.emplace_back(p0, p1);
                //}
                //else {
                  //removeSeg(p1, p0, -1);
                  //vRemovedSegs.emplace_back(p1, p0);
                //}
                //break;
              //}
            //}
            if (p0 < p1) {
              const Int segIdx = _mSegIds.at(std::make_pair(p0Idx, p1Idx));
              vAdjustedSegs.emplace_back(Segment3d<Int>(p0, p1), _vSegData.at(segIdx), false);
              removeSeg(p0, p1, -4);
            }
            else {
              const Int segIdx = _mSegIds.at(std::make_pair(p1Idx, p0Idx));
              vAdjustedSegs.emplace_back(Segment3d<Int>(p1, p0), _vSegData.at(segIdx), false);
              removeSeg(p1, p0, -4);
            }

//#ifndef NDEBUG
            //Vector<Int> vP0TouchedSegs;
            //_spatialSegs.query(p0, p0, vP0TouchedSegs);
            //assert(vP0TouchedSegs.size() == 0);
//#endif
            auto getDim = [] (const Point3d<Int>& p0, const Point3d<Int>& p1) -> Byte {
              if (p0.x() != p1.x()) return 0;
              if (p0.y() != p1.y()) return 1;
              if (p0.z() != p1.z()) return 2;
              assert(false);
            };

            auto mergeSegs = [&] (const Point3d<Int>& pt) {
              Vector<Int> vPinIds;
              _spatialPins.query(pt, pt, vPinIds);
              assert(vPinIds.size() <= 1);
              for (const Int pinIdx : vPinIds) {
                for (const auto& acs : _vpPins.at(pinIdx)->vAcs()) {
                  if (acs == pt) {
                    return;
                  }
                }
              }

              Vector<Int> vTouchedSegs;
              _spatialSegs.query(pt, pt, vTouchedSegs);

              Array<Vector<Int>, 3> arr;
              for (const Int segIdx : vTouchedSegs) {
                const auto& s = _vSegs.at(segIdx);
                const auto& p0 = _vPoints.at(s.first);
                const auto& p1 = _vPoints.at(s.second);
                const Int dim = getDim(p0, p1);
                arr.at(dim).emplace_back(segIdx);
              }
              const Int nonEmptyDim = !arr.at(0).empty() + !arr.at(1).empty() + !arr.at(2).empty();
              if (nonEmptyDim > 1) {
                return;
              }
              //for (Int dim = 0; dim < 3; ++dim) {
              for (Int dim = 0; dim < 2; ++dim) { // skip vias merging
                const auto& vSegs = arr.at(dim);
                assert(vSegs.size() <= 2);
                if (vSegs.size() < 2) {
                  continue;
                }
                if (!vSegs.empty()) {
                  const auto& sd = _vSegData.at(vSegs.at(0));
//#ifndef NDEBUG
                  //for (size_t i = 1; i < vSegs.size(); ++i) {
                    //const Int s0 = vSegs.at(i - 1);
                    //const Int s1 = vSegs.at(i);
                    //if (_vSegData.at(s0) != _vSegData.at(s1)) {
                      //const auto& p00 = _vPoints.at(_vSegs.at(s0).first);
                      //const auto& p01 = _vPoints.at(_vSegs.at(s0).second);
                      //const auto& p10 = _vPoints.at(_vSegs.at(s1).first);
                      //const auto& p11 = _vPoints.at(_vSegs.at(s1).second);
                      //std::cerr << "s0 " << p00 << p01 << " " << _vSegData.at(s0) << std::endl;
                      //std::cerr << "s1 " << p10 << p11 << " " << _vSegData.at(s1) << std::endl;
                    //}
                    //assert(_vSegData.at(s0) == _vSegData.at(s1));
                  //}
//#endif
                  Int newP0Idx = -1, newP1Idx = -1;
                  Int low = MAX_INT, high = MIN_INT;
                  for (const Int segIdx : vSegs) {
                    const Int p0Idx = _vSegs.at(segIdx).first;
                    const Int p1Idx = _vSegs.at(segIdx).second;
                    const auto& p0 = _vPoints.at(p0Idx);
                    const auto& p1 = _vPoints.at(p1Idx);
                    const Int l = p0.val(dim), h = p1.val(dim);
                    assert(l < h);
                    if (l < low) {
                      low = l;
                      newP0Idx = p0Idx;
                    }
                    if (h > high) {
                      high = h;
                      newP1Idx = p1Idx;
                    }
                    vAdjustedSegs.emplace_back(Segment3d<Int>(p0, p1), _vSegData.at(segIdx), false);
                    removeSeg(p0, p1, segIdx);
                  }
                  assert(newP0Idx >= 0 and newP1Idx >= 0);
                  vAdjustedSegs.emplace_back(Segment3d<Int>(_vPoints.at(newP0Idx), _vPoints.at(newP1Idx)), sd, true);
                  insertSeg(newP0Idx, newP1Idx, sd);
                }
              }
            };
            //mergeSegs(p0);
            mergeSegs(p1);

            --deg[u];
            assert(deg[u] == 0);
            --deg[par[u]];
            if (deg[par[u]] == 1) {
              q.push(par[u]);
            }
          }
        }
      }
    }
  }

  // remove floating
  if (removeFloating) {
    for (size_t i = 0; i < _vPoints.size(); ++i) {
      if (!vis[i]) {
        const Int p0Idx = i;
        const auto& p0 = _vPoints.at(i);
        const auto vAdjPtIds = _vvAdjPtIds.at(i); // do not use reference
        if (!vAdjPtIds.empty()) {
          for (const Int p1Idx : vAdjPtIds) {
            const auto& p1 = _vPoints.at(p1Idx);
            if (p0 < p1) {
              const Int segIdx = _mSegIds.at(std::make_pair(p0Idx, p1Idx));
              vAdjustedSegs.emplace_back(Segment3d<Int>(p0, p1), _vSegData.at(segIdx), false);
              removeSeg(p0, p1, -6);
            }
            else {
              const Int segIdx = _mSegIds.at(std::make_pair(p1Idx, p0Idx));
              vAdjustedSegs.emplace_back(Segment3d<Int>(p1, p0), _vSegData.at(segIdx), false);
              removeSeg(p1, p0, -6);
            }
          }
        }
      }
    }
  }
  // remove redundant
  if (removeRedundant) {
    this->removeRedundant(vAdjustedSegs);
  }
}

void TopoTree::removeFloating()
{
  Vector<Tuple<Segment3d<Int>, SegData, bool>> vAdjustedSegs;
  removeFloating(vAdjustedSegs);
}

void TopoTree::removeFloating(Vector<Tuple<Segment3d<Int>, SegData, bool>>& vAdjustedSegs)
{
  auto sp = _spatialSegs;
  Vector<char> isTerminal(_vPoints.size(), 0);
  Vector<char> vis(_vPoints.size(), 0);
  for (size_t i = 0; i < _vPoints.size(); ++i) {
    isTerminal[i] = bTouchPin(_vPoints[i]);
  }

  std::function<void(const Int)> dfs = [&] (const Int u) {
    if (vis[u]) {
      return;
    }
    vis[u] = 1;

    for (const Int v : _vvAdjPtIds[u]) {
      if (!vis[v]) {
        dfs(v);
      }
    }
  };

  for (size_t i = 0; i < _vPoints.size(); ++i) {
    if (!vis[i] and isTerminal[i]) {
      dfs(i);
    }
  }
  for (size_t i = 0; i < _vPoints.size(); ++i) {
    if (!vis[i]) {
      const Int p0Idx = i;
      const auto& p0 = _vPoints.at(i);
      const auto vAdjPtIds = _vvAdjPtIds.at(i); // do not use reference, since remvoeSeg is called inside
      if (!vAdjPtIds.empty()) {
        for (const Int p1Idx : vAdjPtIds) {
          const auto& p1 = _vPoints.at(p1Idx);
          if (p0 < p1) {
            const Int segIdx = _mSegIds.at(std::make_pair(p0Idx, p1Idx));
            vAdjustedSegs.emplace_back(Segment3d<Int>(p0, p1), _vSegData.at(segIdx), false);
            removeSeg(p0, p1, -3);
          }
          else {
            const Int segIdx = _mSegIds.at(std::make_pair(p1Idx, p0Idx));
            vAdjustedSegs.emplace_back(Segment3d<Int>(p1, p0), _vSegData.at(segIdx), false);
            removeSeg(p1, p0, -3);
          }
        }
      }
    }
  }
}

void TopoTree::removeRedundant()
{
  Vector<Tuple<Segment3d<Int>, SegData, bool>> vAdjustedSegs;
  removeRedundant(vAdjustedSegs);
}

void TopoTree::removeRedundant(Vector<Tuple<Segment3d<Int>, SegData, bool>>& vAdjustedSegs)
{
  Vector<Segment3d<Int>> vRemoves;
  for (const auto& p : _spatialSegs) {
    const Int segIdx = p.second;
    const Int uIdx = _vSegs.at(segIdx).first;
    const Int vIdx = _vSegs.at(segIdx).second;
    const auto& u = _vPoints.at(uIdx);
    const auto& v = _vPoints.at(vIdx);
    Vector<Int> vUPinIds;
    Vector<Int> vVPinIds;
    _spatialPins.query(u, u, vUPinIds);
    _spatialPins.query(v, v, vVPinIds);
    assert(vUPinIds.size() <= 1 and vVPinIds.size() <= 1);
    if (!vUPinIds.empty() and !vVPinIds.empty()) {
      if (vUPinIds.at(0) == vVPinIds.at(0)) {
        const Segment3d<Int> s = u < v ? Segment3d<Int>(u, v) : Segment3d<Int>(v, u);
        vRemoves.emplace_back(s);
        vAdjustedSegs.emplace_back(s, _vSegData.at(segIdx), false);
      }
    }
  }
  for (const auto& s : vRemoves) {
    removeSeg(s.p0(), s.p1(), -7);
  }
}

bool TopoTree::isConnected() const
{
  Vector<char> vis(_vPoints.size(), 0);

  DisjointSet ds(_vpPins.size());

  std::function<void(const Int, const Int)> dfs = [&] (const Int u, const Int pinIdx) {
    if (vis[u]) {
      return;
    }
    vis[u] = true;

    const auto& pt = _vPoints.at(u);
    Vector<Int> touchPins;
    _spatialPins.query(pt, pt, touchPins);
    assert(touchPins.size() == 0 or touchPins.size() == 1);
    if (touchPins.size() == 1) {
      ds.merge(pinIdx, touchPins[0]);
    }
    

    for (const Int v : _vvAdjPtIds[u]) {
      if (!vis[v]) {
        dfs(v, pinIdx);
      }
    }
  };
  for (const auto& p : _spatialSegs) {
    const Int segIdx = p.second;
    const Int uIdx = _vSegs.at(segIdx).first;
    const auto& u = _vPoints.at(uIdx);

    Vector<Int> touchPins;
    _spatialPins.query(u, u, touchPins);
    assert(touchPins.size() == 0 or touchPins.size() == 1);
    if (!vis[uIdx] and touchPins.size() == 1) {
      dfs(uIdx, touchPins[0]);
      break;
    }

    const Int vIdx = _vSegs.at(segIdx).second;
    const auto& v = _vPoints.at(vIdx);

    touchPins.clear();
    _spatialPins.query(v, v, touchPins);
    assert(touchPins.size() == 0 or touchPins.size() == 1);

    if (!vis[vIdx] and touchPins.size() == 1) {
      dfs(vIdx, touchPins[0]);
      break;
    }
  }
  return ds.nSets() == 1;
}

void TopoTree::refineSelfLoop()
{
  Vector<Tuple<Segment3d<Int>, SegData, bool>> vAdjustedSegs;
  refineSelfLoop(vAdjustedSegs);
}

void TopoTree::refineSelfLoop(Vector<Tuple<Segment3d<Int>, SegData, bool>>& vAdjustedSegs)
{
  bool bContinue = true;
  while (bContinue) {
    Vector<Int> par(_vPoints.size(), -1);
    Vector<char> isTerminal(_vPoints.size(), 0);
    Vector<char> color(_vPoints.size(), 0);
    assert(_vvAdjPtIds.size() == _vPoints.size());
    for (size_t i = 0; i < _vPoints.size(); ++i) {
      isTerminal[i] = bTouchPin(_vPoints[i]);
    }
    bool bHasCycle = false;
    for (size_t i = 0; i < _vPoints.size(); ++i) {
      if (color[i] == 0 and isTerminal[i]) {
        Vector<Vector<Pair<Int, Int>>> vCycles;
        dfsCycle(i, i, par, color, vCycles);
        if (vCycles.size() > 0) {
          bHasCycle = true;
        }
        for (const auto& v : vCycles) {
          Int longestEdgeIdx = -1, maxLength = MIN_INT;
          for (size_t j = 0; j < v.size(); ++j) {
            const Int p0Idx = v[j].first;
            const Int p1Idx = v[j].second;
            const auto& p0 = _vPoints[p0Idx];
            const auto& p1 = _vPoints[p1Idx];
            const Segment3d<Int> seg(p0, p1);
            const Int length = seg.length();
            if (length > maxLength) {
              longestEdgeIdx = j;
              maxLength = length;
            }
          }
          assert(longestEdgeIdx != -1);
          const Int p0Idx = v[longestEdgeIdx].first;
          const Int p1Idx = v[longestEdgeIdx].second;
          const auto& p0 = _vPoints[p0Idx];
          const auto& p1 = _vPoints[p1Idx];
          const Int segIdx = _mSegIds.at(std::make_pair(p0Idx, p1Idx));
          const Segment3d<Int> seg(p0, p1);
          Vector<Int> vSegIds;
          if (p0 < p1)
            _spatialSegs.query(p0, p1, vSegIds);
          else
            _spatialSegs.query(p1, p0, vSegIds);
          bool bRemoved = true;
          for (const Int segIdx : vSegIds) {
            const auto& pp0 = _vPoints[_vSegs[segIdx].first];
            const auto& pp1 = _vPoints[_vSegs[segIdx].second];
            if (p0 < p1) {
              if (pp0 == p0 and pp1 == p1) {
                bRemoved = false;
                break;
              }
            }
            else {
              if (pp0 == p1 and pp1 == p0) {
                bRemoved = false;
                break;
              }
            }
          }
          if (bRemoved)
            continue;
          for (Int dim = 0; dim < 3; ++dim) {
            if (p0.val(dim) != p1.val(dim)) {
              if (p0 < p1) {
                removeSeg(p0, p1, -5);
                vAdjustedSegs.emplace_back(Segment3d<Int>(p0, p1), _vSegData.at(segIdx), false);
              }
              else {
                removeSeg(p1, p0, -5);
                vAdjustedSegs.emplace_back(Segment3d<Int>(p1, p1), _vSegData.at(segIdx), false);
              }
              break;
            }
          }
        }
      }
    }
    bContinue = bHasCycle;
  }
}

void TopoTree::vCompSegs(const Int compIdx, Vector<Segment3d<Int>>& vSegs) const
{
  const auto& comp = _vsCompSegs.at(compIdx);
  for (auto it = comp.cbegin(); it != comp.end(); ++it) {
    const Int segIdx = *it;
    const auto& ptIdxPair = _vSegs.at(segIdx);
    const auto& p0 = _vPoints.at(ptIdxPair.first);
    const auto& p1 = _vPoints.at(ptIdxPair.second);
    vSegs.emplace_back(p0, p1);
  }
}

void TopoTree::vCompSegs(const Int compIdx, Vector<Segment3d<Int>>& vSegs, Vector<SegData>& vSegData) const
{
  const auto& comp = _vsCompSegs.at(compIdx);
  for (auto it = comp.cbegin(); it != comp.end(); ++it) {
    const Int segIdx = *it;
    const auto& ptIdxPair = _vSegs.at(segIdx);
    const auto& segData = _vSegData.at(segIdx);
    const auto& p0 = _vPoints.at(ptIdxPair.first);
    const auto& p1 = _vPoints.at(ptIdxPair.second);
    vSegs.emplace_back(p0, p1);
    vSegData.emplace_back(segData);
  }
}

void TopoTree::vCompPts(const Int compIdx, Vector<Point3d<Int>>& vPts) const
{
  const auto& comp = _vsCompPts.at(compIdx);
  std::copy(comp.cbegin(), comp.cend(), std::back_inserter(vPts));
}

void TopoTree::addPinAcs()
{
  for (size_t i = 0; i < _vpPins.size(); ++i) {
    const Pin& pin = *_vpPins.at(i);
    const Vector<Point3d<Int>>& vAcs = pin.vAcs();

    FlatHashMap<Pair<Int, Int>, Vector<Point3d<Int>>> sameXY;
    FlatHashMap<Pair<Point3d<Int>, Point3d<Int>>, Vector<Int>> sameBoxAcs;
    for (const Point3d<Int>& acs : vAcs) {
      const Int idx = addPoint(acs);

      Vector<spatial3d::b_box<Int>> vBoxes;
      _spatialPins.queryBox(acs, acs, vBoxes);
      assert(vBoxes.size() == 1);
      const auto& b = vBoxes.at(0);
      const auto& p = std::make_pair(b.min_corner(), b.max_corner());
      if (sameBoxAcs.find(p) == sameBoxAcs.end()) {
        sameBoxAcs.emplace(p, Vector<Int>());
      }
      sameBoxAcs[p].emplace_back(idx);


      const Pair<Int, Int> xy = std::make_pair(acs.x(), acs.y());
      if (sameXY.find(xy) == sameXY.end()) {
        sameXY.emplace(xy, Vector<Point3d<Int>>());
      }
      sameXY[xy].emplace_back(acs);
    }
    for (auto& item : sameBoxAcs) {
      auto& v = item.second;
      FlatHashMap<Int, Vector<Int>> sameXIds;
      FlatHashMap<Int, Vector<Int>> sameYIds;
      for (const Int ptIdx : v) {
        const auto& pt = _vPoints.at(ptIdx);
        if (sameXIds.find(pt.x()) == sameXIds.end()) {
          sameXIds.emplace(pt.x(), Vector<Int>());
        }
        if (sameYIds.find(pt.y()) == sameYIds.end()) {
          sameYIds.emplace(pt.y(), Vector<Int>());
        }
        sameXIds[pt.x()].emplace_back(ptIdx);
        sameYIds[pt.y()].emplace_back(ptIdx);
      }
      for (auto& it : sameXIds) {
        auto& vv = it.second;
        std::sort(vv.begin(), vv.end(),
                  [&] (const Int i0, const Int i1) {
                    return _vPoints.at(i0).y() < _vPoints.at(i1).y();
                  });
        for (size_t j = 1; j < vv.size(); ++j) {
          const Int preIdx = vv.at(j - 1);
          const Int curIdx = vv.at(j);
          _vvAdjPtIds[preIdx].emplace_back(curIdx);
          _vvAdjPtIds[curIdx].emplace_back(preIdx);
        }
      }
      for (auto& it : sameYIds) {
        auto& vv = it.second;
        std::sort(vv.begin(), vv.end(),
                  [&] (const Int i0, const Int i1) {
                    return _vPoints.at(i0).x() < _vPoints.at(i1).x();
                  });
        for (size_t j = 1; j < vv.size(); ++j) {
          const Int preIdx = vv.at(j - 1);
          const Int curIdx = vv.at(j);
          _vvAdjPtIds[preIdx].emplace_back(curIdx);
          _vvAdjPtIds[curIdx].emplace_back(preIdx);
        }
      }
    }
    for (auto& item : sameXY) {
      auto& v = item.second;
      std::sort(v.begin(), v.end(),
                [] (const auto& p0, const auto& p1) {
                  return p0.z() < p1.z();
                });
      for (size_t j = 1; j < v.size(); ++j) {
        const auto& pre = v.at(j - 1);
        const auto& cur = v.at(j);
        if (pre.z() + 2 == cur.z()) {
          const Int preIdx = _mPt2Idx.at(pre);
          const Int curIdx = _mPt2Idx.at(cur);
          _vvAdjPtIds[preIdx].emplace_back(curIdx);
          _vvAdjPtIds[curIdx].emplace_back(preIdx);
        }
      }
    }
  }
}

void TopoTree::groupComps()
{
  DisjointSet ds(_vPoints.size());
  Vector<Vector<Int>> vvPinAdjs(_vpPins.size());
  calcConnectivity(ds, vvPinAdjs);

  auto add2Map = [] (FlatHashMap<Int, Int>& m, const Int key) {
    if (m.find(key) == m.end()) {
      m.emplace(key, m.size());
    }
  };

  FlatHashMap<Int, Int> mPt2CompIdx;
  for (auto it = _spatialSegs.begin(); it != _spatialSegs.end(); ++it) {
    const Pair<Int, Int>& ptPair = _vSegs[it->second];
    add2Map(mPt2CompIdx, ds.find(ptPair.first));
    add2Map(mPt2CompIdx, ds.find(ptPair.second));
  }
  for (size_t i = 0; i < _vpPins.size(); ++i) {
    Pin* pPin = _vpPins.at(i);
    for (const Point3d<Int>& pt : pPin->vAcs()) {
      add2Map(mPt2CompIdx, ds.find(_mPt2Idx.at(pt)));
    }
  }

  _vsCompPts.assign(mPt2CompIdx.size(), FlatHashSet<Point3d<Int>>());
  _vsCompSegs.assign(mPt2CompIdx.size(), FlatHashSet<Int>());

  Int xl = MAX_INT, yl = MAX_INT;
  Int xh = MIN_INT, yh = MIN_INT;
  for (auto it = _spatialSegs.begin(); it != _spatialSegs.end(); ++it) {
    const Int segIdx = it->second;
    const Pair<Int, Int>& ptPair = _vSegs[segIdx];
    const Int rootIdx = ds.find(ptPair.first);
    assert(rootIdx == ds.find(ptPair.second));
    
    const Int compIdx = mPt2CompIdx.at(rootIdx);
    assert(_vsCompSegs[compIdx].find(segIdx) == _vsCompSegs[compIdx].end());

    const Point3d<Int>& p0 = _vPoints.at(ptPair.first);
    const Point3d<Int>& p1 = _vPoints.at(ptPair.second);

    xl = std::min({xl, p0.x(), p1.x()});
    yl = std::min({yl, p0.y(), p1.y()});
    xh = std::max({xh, p0.x(), p1.x()});
    yh = std::max({yh, p0.y(), p1.y()});
    
    
    //for (Int dim = 0; dim <= 2; ++dim) {
      //if (p0.val(dim) != p1.val(dim)) {
        //Int low, high;
        //Point3d<Int> pt;
        //if (p0 < p1) {
          //pt = p0;
          //low = p0.val(dim);
          //high = p1.val(dim);
        //}
        //else {
          //pt = p1;
          //low = p1.val(dim);
          //high = p0.val(dim);
        //}
        //for (Int d = low; d <= high; ++d) {
          //pt.val(dim) = d;
          ////std::cerr << pt << std::endl;
          //_vsCompPts[compIdx].emplace(pt);
        //}
        //break;
      //}
    //}
    _vsCompSegs[compIdx].emplace(segIdx);
  }
  _bbox.set(xl, yl, xh, yh);
  //std::cerr << _bbox << std::endl;

  for (size_t i = 0; i < _vpPins.size(); ++i) {
    Pin* pPin = _vpPins.at(i);
    for (const Point3d<Int>& pt : pPin->vAcs()) {
      const Int ptIdx = _mPt2Idx.at(pt);
      const Int rootIdx = ds.find(ptIdx);
      const Int compIdx = mPt2CompIdx.at(rootIdx);
      _vsCompPts[compIdx].emplace(pt);
    }
  }
}

void TopoTree::showComps() const
{
  std::cerr << " NumComps: " << _vsCompSegs.size() << std::endl;
  for (size_t i = 0; i < _vsCompSegs.size(); ++i) {
    std::cerr << "Comp " << i << " ";
    std::cerr<< "(Seg: " << _vsCompSegs[i].size() << ", ";
    std::cerr << "Pt: " << _vsCompPts[i].size() << ")" << std::endl;
    if (_vsCompSegs[i].size() == 0) {
      for (const auto& p : _vsCompPts[i]) {
        std::cerr << "pt " << p << std::endl;
      }
    }
    else {
      for (const Int segIdx : _vsCompSegs[i]) {
        const auto& p0 = _vPoints[_vSegs[segIdx].first];
        const auto& p1 = _vPoints[_vSegs[segIdx].second];
        std::cerr << "seg " << p0 << p1 << " ";

        const auto& segData = _vSegData[segIdx];
        std::cerr << (segData.pVia ? segData.pVia->name() : "nullptr") << " " << util::enumUtil::val2Str(Orient2dEStr, segData.orient) << " " << segData.width << " "  << segData.ext <<std::endl;
      }
    }
  }
  std::cerr << "Connected: " << isConnected() << std::endl << std::endl;
  //std::cerr << std::endl;
  //for (size_t i = 0; i < _vSegs.size(); ++i ) {
      //const auto& p0 = _vPoints[_vSegs[i].first];
      //const auto& p1 = _vPoints[_vSegs[i].second];
      //std::cerr << p0 << p1 << " ";

      //const auto& segData = _vSegData[i];
      //std::cerr << segData.pVia << " " << util::enumUtil::val2Str(Orient2dEStr, segData.orient) << " " << segData.width << " "  << segData.ext <<std::endl;
  //}
  //for (size_t i = 0; i < _vsCompPts.size(); ++i) {
    //std::cerr << "Comp " << i << std::endl;
    //for (const auto& pt : _vsCompPts[i]) {
      //std::cerr << pt;
    //}
    //std::cerr << std::endl;
  //}
}

void TopoTree::clear()
{
  _vpPins.clear();
  _vPoints.clear();
  _vvAdjPtIds.clear();
  _vSegs.clear();
  _vSegData.clear();
  _vsCompPts.clear();
  _vsCompSegs.clear();
  _mPt2Idx.clear();
  _mSegIds.clear();
  _spatialSegs.clear();
  _spatialPins.clear();
  _length = 0;
  _length2d = 0;
  
  _vpPins.shrink_to_fit();
  _vPoints.shrink_to_fit();
  _vSegs.shrink_to_fit();
  _vSegData.shrink_to_fit();
  _vvAdjPtIds.shrink_to_fit();
  _vsCompPts.shrink_to_fit();
  _vsCompSegs.shrink_to_fit();
}

Int TopoTree::addPoint(const Point3d<Int>& pt)
{
  if (_mPt2Idx.find(pt) != _mPt2Idx.end()) {
    return _mPt2Idx.at(pt);
  }
  else {
    const Int idx = _vPoints.size();
    _mPt2Idx.emplace(pt, idx);
    _vPoints.emplace_back(pt);
    _vvAdjPtIds.emplace_back();
    return idx;
  }
}

Int TopoTree::addSeg(const Pair<Int, Int>& p, const SegData& segData)
{
  //assert(segData.pVia == nullptr);
  if (_mSegIds.find(p) != _mSegIds.end()) {
    return _mSegIds.at(p);
  }
  else {
    _mSegIds.emplace(p, _vSegs.size());
    _vSegs.emplace_back(p);
    _vSegData.emplace_back(segData);
    return _mSegIds.size() - 1;
  }
}

void TopoTree::insertSeg(const Int p0Idx, const Int p1Idx, const SegData& segData)
{
  Int segIdx = addSeg({p0Idx, p1Idx}, segData);
  assert(0 <= segIdx and segIdx < static_cast<Int>(_vSegs.size()));

  const auto& p0 = _vPoints.at(p0Idx);
  const auto& p1 = _vPoints.at(p1Idx);
  assert(p0.x() <= p1.x() and p0.y() <= p1.y() and p0.z() <= p1.z());

  auto p0IdxIt = _mPt2Idx.find(p0);
  auto p1IdxIt = _mPt2Idx.find(p1);
  assert(p0IdxIt != _mPt2Idx.end());
  assert(p1IdxIt != _mPt2Idx.end());

  _vvAdjPtIds[p0IdxIt->second].emplace_back(p1Idx);
  _vvAdjPtIds[p1IdxIt->second].emplace_back(p0Idx);
  _spatialSegs.insert(p0, p1, segIdx);

  assert(p0 == _vPoints[_vSegs[segIdx].first]);
  assert(p1 == _vPoints[_vSegs[segIdx].second]);

  //if (_pNet) {
    //_pNet->mArcs().emplace(Segment3d<Int>(p0, p1), segData);
  //}
  //if (_pRo) {
    //_pRo->arcs().emplace(Segment3d<Int>(p0, p1));
  //}
  
  // update wl
  const Int dis = Point3d<Int>::Mdistance(p0, p1);
  _length += dis;
  _length2d += (p0.z() == p1.z() ? dis : 0);
}

bool TopoTree::removeSeg(const Point3d<Int>& p0, const Point3d<Int>& p1, const Int segIdx)
{
  assert(p0.x() <= p1.x() and p0.y() <= p1.y() and p0.z() <= p1.z());
  const bool b = _spatialSegs.erase(p0, p1, segIdx);
  //if (!b) {
    ////return false;
    //std::cerr << p0 << p1 << " " << segIdx << std::endl;
    //for (const auto& p : _spatialSegs) {
      //const Int segIdx = p.second;
      //const auto& seg = _vSegs.at(segIdx);
      //std::cerr << _vPoints.at(seg.first) << _vPoints.at(seg.second) << std::endl;
    //}
  //}
  assert(b);
  //if (_pNet) {
    //const bool b = _pNet->mArcs().erase(Segment3d<Int>(p0, p1));
    //assert(b);
  //}
  //if (_pRo) {
    //const bool b = _pRo->arcs().erase(Segment3d<Int>(p0, p1));
    //assert(b);
  //}

  auto p0IdxIt = _mPt2Idx.find(p0);
  auto p1IdxIt = _mPt2Idx.find(p1);
  assert(p0IdxIt != _mPt2Idx.end());
  assert(p1IdxIt != _mPt2Idx.end());
  
  const Int p0Idx = p0IdxIt->second;
  const Int p1Idx = p1IdxIt->second;
  Vector<Int>& vAdj0 = _vvAdjPtIds[p0Idx];
  Vector<Int>& vAdj1 = _vvAdjPtIds[p1Idx];
  
  auto it01 = std::find(vAdj0.begin(), vAdj0.end(), p1Idx);
  auto it10 = std::find(vAdj1.begin(), vAdj1.end(), p0Idx);
  assert(it01 != vAdj0.end());
  assert(it10 != vAdj1.end());
  vAdj0.erase(it01);
  vAdj1.erase(it10);

  // update wl
  const Int dis = Point3d<Int>::Mdistance(p0, p1);
  _length -= dis;
  _length2d -= (p0.z() == p1.z() ? dis : 0);

  return true;
}

bool TopoTree::bTouchPin(const Point3d<Int>& pt) const
{
  return _spatialPins.exist(pt, pt);
}

bool TopoTree::bTouchPin(const Point3d<Int>& pt, const Pin& pin) const
{
  Vector<Int> v;
  _spatialPins.query(pt, pt, v);
  if (v.empty()) {
    return false;
  }
  //if (v.size() != 1) {
    //std::cerr << v.size() << std::endl;
    //for (const Int i : v) {
      //std::cerr << _vpPins.at(i)->name() << std::endl;
    //}
  //} 
  assert(v.size() == 1);
  return _vpPins.at(v.at(0))->idx() == pin.idx();
}

void TopoTree::dfsDeg(const Int u, const Int p, Vector<Int>& par, Vector<char>& vis, Vector<char>& deg, Queue<Int>& q) const
{
  if (vis[u]) {
    return;
  }
  vis[u] = 1;
  if (deg[u] == 1) {
    q.push(u);
  }
  par[u] = p;
  for (const Int v : _vvAdjPtIds[u]) {
    if (v != p) {
      dfsDeg(v, u, par, vis, deg, q);
    }
  }
}

void TopoTree::dfsCycle(const Int u, const Int p, Vector<Int>& par, Vector<char>& color, Vector<Vector<Pair<Int, Int>>>& vvEdges) const
{
  if (color[u] == 2) {
    return;
  }
  if (color[u] == 1) {
    vvEdges.emplace_back();
    Int cur = p;
    while (cur != u) {
      vvEdges.back().emplace_back(cur, par[cur]);
      cur = par[cur];
    }
    return;
  }
  par[u] = p;
  color[u] = 1;
  for (const Int v : _vvAdjPtIds[u]) {
    if (v != p) {
      dfsCycle(v, u, par, color, vvEdges);
    }
  }
  color[u] = 2;
}

void TopoTree::dfsPaths(const Int u, const Int t, Vector<Int>& prefix, Vector<char>& vis, Vector<Vector<Int>>& paths) const
{
  vis[u] = true;
  prefix.emplace_back(u);

  if (u == t) { 
    paths.emplace_back(prefix);
  }
  else {
    for (const Int v : _vvAdjPtIds[u]) {
      if (!vis[v]) {
        dfsPaths(v, t, prefix, vis, paths);
      }
    }
  }

  prefix.pop_back();
  vis[u] = false;
}

void TopoTree::dfsPins(const Int u, Vector<char>& vis, FlatHashSet<Pin*>& pins) const
{
  vis[u] = true;

  Vector<Int> vPinIds;
  const Point3d<Int> pt = _vPoints.at(u);
  _spatialPins.query(pt, pt, vPinIds);
  if (!vPinIds.empty()) {
    assert(vPinIds.size() == 1);
    pins.emplace(_vpPins.at(vPinIds[0]));
  }
  for (const Int v : _vvAdjPtIds[u]) {
    if (!vis[v]) {
      dfsPins(v, vis, pins);
    }
  }
}

void TopoTree::vPaths(const Point3d<Int>& s, const Point3d<Int>& t, Vector<Vector<Segment3d<Int>>>& vvSegs) const
{
  if (_mPt2Idx.find(s) == _mPt2Idx.end()) {
    return;
  }
  if (_mPt2Idx.find(t) == _mPt2Idx.end()) {
    return;
  }

  Vector<char> vis(_vPoints.size(), false);
  Vector<Int> prefix;
  Vector<Vector<Int>> paths;

  const Int sIdx = _mPt2Idx.at(s);
  const Int tIdx = _mPt2Idx.at(t);

  dfsPaths(sIdx, tIdx, prefix, vis, paths);

  vvSegs.reserve(paths.size());
  for (const auto& path : paths) {
    vvSegs.emplace_back();
    vvSegs.back().reserve(path.size());
    auto u0 = path.begin();
    auto v0 = std::next(u0, 1);
    auto v1 = std::next(v0, 1);
    auto needMerge = [&] (const auto& u0, const auto& v0, const auto& v1) -> bool {
      const auto& u0p = _vPoints.at(*u0);
      const auto& v0p = _vPoints.at(*v0);
      const auto& v1p = _vPoints.at(*v1);
      if (u0p.z() != v0p.z()) {
        return false;
      }
      if (!(bTouchPin(u0p) and bTouchPin(v0p) and bTouchPin(v1p))) {
        return false;
      }
      return direction3d::findDir(u0p, v0p) == direction3d::findDir(v0p, v1p);
    };

    auto addSeg = [&] (const auto& u0p, const auto& v0p) {
      if (u0p < v0p) {
        vvSegs.back().emplace_back(u0p, v0p);
      }
      else {
        vvSegs.back().emplace_back(v0p, u0p);
      }
    };

    for (; v1 != path.end(); ++v1) {
      if (needMerge(u0, v0, v1)) {
        v0 = v1;
      }
      else {
        addSeg(_vPoints.at(*u0), _vPoints.at(*v0));
        u0 = v0;
        v0 = v1;
      }
    }
    addSeg(_vPoints.at(*u0), _vPoints.at(*v0));
  }

}

//void TopoTree::vPaths(const Point3d<Int>& s, const Point3d<Int>& t, Vector<Vector<Segment3d<Int>>>& vvSegs, Vector<Vector<SegData>>& vvSegData) const
//{
  //if (_mPt2Idx.find(s) == _mPt2Idx.end()) {
    //return;
  //}
  //if (_mPt2Idx.find(t) == _mPt2Idx.end()) {
    //return;
  //}

  //Vector<char> vis(_vPoints.size(), false);
  //Vector<Int> prefix;
  //Vector<Vector<Int>> paths;

  //const Int sIdx = _mPt2Idx.at(s);
  //const Int tIdx = _mPt2Idx.at(t);

  //dfsPaths(sIdx, tIdx, prefix, vis, paths);

  //for (const auto& path : paths) {
    //vvSegs.emplace_back();
    //vvSegData.emplace_back();
    //auto u0 = path.begin();
    //auto v0 = std::next(u0, 1);
    //auto v1 = std::next(v0, 1);
    //auto needMerge = [&] (const auto& u0p, const auto& v0p, const auto& u1p, const auto& v1p) -> bool {
      //if (u0p.z() != v0p.z()) {
        //return false;
      //}
      //return direction3d::findDir(u0p, v0p) == direction3d::findDir(u1p, v1p);
    //};

    //std::cerr << "lului " << path.size() << std::endl; 
    //for (; v1 != path.end(); ++v1) {
      //const Int u0Idx = *u0;
      //const Int v0Idx = *v0;
      //const Int v1Idx = *v1;
      //const auto& u0p = _vPoints.at(u0Idx);
      //const auto& v0p = _vPoints.at(v0Idx);
      //const auto& v1p = _vPoints.at(v1Idx);
      //assert(Segment3d<Int>(u0p, v0p).b90());
      //if (needMerge(u0p, v0p, v0p, v1p)) {
        //v0 = v1;
      //}
      //else {
        //if (u0p < v0p) {
          //vvSegs.back().emplace_back(u0p, v0p);
          //vvSegData.back().emplace_back(_vSegData.at(_mSegIds.at({u0Idx, v0Idx})));
        //}
        //else {
          //vvSegs.back().emplace_back(v0p, u0p);
          //vvSegData.back().emplace_back(_vSegData.at(_mSegIds.at({v0Idx, u0Idx})));
        //}
        //u0 = v0;
        //v0 = v1;
      //}
    //}
  //}
//}
  
  
void TopoTree::vPaths(const Vector<Point3d<Int>>& vs, const Vector<Point3d<Int>>& vt, Vector<Vector<Segment3d<Int>>>& vvSegs) const
{
  for (const auto& s : vs) {
    for (const auto& t : vt) {
      vPaths(s, t, vvSegs);
    }
  }
}

//void TopoTree::vPaths(const Vector<Point3d<Int>>& vs, const Vector<Point3d<Int>>& vt, Vector<Vector<Segment3d<Int>>>& vvSegs, Vector<Vector<SegData>>& vvSegData) const
//{
  //for (const auto& s : vs) {
    //for (const auto& t : vt) {
      //vPaths(s, t, vvSegs, vvSegData);
    //}
  //}
//}

void TopoTree::vPaths(const Pin& sPin, const Pin& tPin, Vector<Vector<Segment3d<Int>>>& vvSegs) const
{
  Vector<Point3d<Int>> vs, vt;

  auto addPt = [&] (const auto& p) {
    if (bTouchPin(p, sPin)) {
      vs.emplace_back(p);
    }
    else if (bTouchPin(p, tPin)) {
      vt.emplace_back(p);
    }
  };

  for (auto it = _spatialSegs.begin(); it != _spatialSegs.end(); ++it) {
    const auto& ptPair = _vSegs.at(it->second);
    const auto& u = _vPoints[ptPair.first];
    const auto& v = _vPoints[ptPair.second];
    addPt(u);
    addPt(v);
  }
  vPaths(vs, vt, vvSegs);

  if (vvSegs.empty()) {
    FlatHashSet<Pin*> sReachablePins, tReachablePins;
    for (const auto& p : vs) {
      Vector<char> vis(_vPoints.size(), false);
      dfsPins(_mPt2Idx.at(p), vis, sReachablePins);
    }
    for (const auto& p : vt) {
      Vector<char> vis(_vPoints.size(), false);
      dfsPins(_mPt2Idx.at(p), vis, tReachablePins);
    }
    const Pin* pMidPin = nullptr;
    for (const auto& pPin : sReachablePins) {
      if (tReachablePins.find(pPin) != tReachablePins.end()) {
        pMidPin = pPin;
        break;
      }
    }
    if (pMidPin) {
      Vector<Vector<Segment3d<Int>>> vPaths0, vPaths1;
      vPaths(sPin, *pMidPin, vPaths0);
      vPaths(*pMidPin, tPin, vPaths1);
      for (const auto& v0 : vPaths0) {
        for (const auto& v1 : vPaths1) {
          vvSegs.emplace_back();
          vvSegs.back() = v0;
          vvSegs.back().insert(vvSegs.back().end(), v1.begin(), v1.end());
        }
      }
    }
    else {
      assert(!isConnected());
    }
  }
}

//void TopoTree::vPaths(const Pin& sPin, const Pin& tPin, Vector<Vector<Segment3d<Int>>>& vvSegs, Vector<Vector<SegData>>& vvSegData) const
//{
  //Vector<Point3d<Int>> vs, vt;

  //auto addPt = [&] (const auto& p) {
    //if (bTouchPin(p, sPin)) {
      //vs.emplace_back(p);
    //}
    //else if (bTouchPin(p, tPin)) {
      //vt.emplace_back(p);
    //}
  //};

  //for (auto it = _spatialSegs.begin(); it != _spatialSegs.end(); ++it) {
    //const auto& ptPair = _vSegs.at(it->second);
    //const auto& u = _vPoints[ptPair.first];
    //const auto& v = _vPoints[ptPair.second];
    //addPt(u);
    //addPt(v);
  //}
  //vPaths(vs, vt, vvSegs, vvSegData);

//}

void TopoTree::calcConnectivity(DisjointSet& ds, Vector<Vector<Int>>& vvPinAdjs) const
{
  //assert(vvPinAdjs.size() > 0);
  for (auto it = _spatialSegs.begin(); it != _spatialSegs.end(); ++it) {
    const auto& ptPair = _vSegs.at(it->second);
    const auto& p0 = _vPoints[ptPair.first];
    const auto& p1 = _vPoints[ptPair.second];
    ds.merge(ptPair.first, ptPair.second);

    Vector<Int> vPinIds;
    _spatialPins.query(p0, p0, vPinIds);
    _spatialPins.query(p1, p1, vPinIds);

    std::sort(vPinIds.begin(), vPinIds.end());
    vPinIds.resize(std::unique(vPinIds.begin(), vPinIds.end()) - vPinIds.begin());

    for (const Int pinIdx : vPinIds) {
      vvPinAdjs[pinIdx].emplace_back(ptPair.first);
    }
  }
  for (size_t i = 0; i < vvPinAdjs.size(); ++i) {
    for (size_t j = 1; j < vvPinAdjs[i].size(); ++j) {
      ds.merge(vvPinAdjs[i][j - 1], vvPinAdjs[i][j]);
    }
  }
  // merge acs connectivities
  for (const Pin* pPin : _vpPins) {
    const Vector<Point3d<Int>>& vAcs = pPin->vAcs();
    for (size_t i = 1; i < vAcs.size(); ++i) {
      ds.merge(_mPt2Idx.at(vAcs[i - 1]), _mPt2Idx.at(vAcs[i]));
    }
  }
}

void TopoTree::init()
{
  buildSpatialPins();
  addPinAcs();
}

void Routable::init()
{

  std::function<void(Routable& ro)>
  flat2Pins = [&] (Routable& ro) {
    assert(ro.pNet() == _pNet);
    for (Pin* pPin : ro.vpPins()) {
      _topo.addPinPtr(pPin);
    }
    for (const Int routableIdx : ro.vRoutableIds()) {
      Routable& childRo = ro.net().routable(routableIdx);
      flat2Pins(childRo);
    }
  };
  flat2Pins(*this);

  _topo.init();
  _topo.setRoPtr(this);
  _topo.setNetPtr(_pNet);
}

void Routable::clearRouting()
{
  //_vWireIds.clear();
  _sWires.clear();
  //_vArcIds.clear();
  _sArcs.clear();
  _topo.clear();

  init();

  _isRouted = false;
}

void Routable::addRoute(const Arc& arc, const ArcData& arcData)
{
  Vector<Tuple<Arc, ArcData, bool>> vAdjustedArcs;
  //if (!arc.b90()) {
    //std::cerr << "ro" << std::endl;
    //std::cerr << arc << std::endl;
    //std::cerr << _pNet->name() << std::endl;
  //}
  _topo.addRoute(arc, arcData, vAdjustedArcs);

  for (const auto& p : vAdjustedArcs) {
    const auto& arc = std::get<0>(p);
    //const auto& arcData  = std::get<1>(p);
    const bool isInsert = std::get<2>(p);
    if (isInsert) {
      _sArcs.emplace(arc);
    }
    else{
      const bool b = _sArcs.erase(arc);
      assert(b);
    }
  }
  assert(_topo.numSegs() == this->numArcs());
}

bool Routable::removeRoute(const Arc& arc)
{
  Vector<Tuple<Arc, ArcData, bool>> vAdjustedArcs;
  const bool b = _topo.removeRoute(arc, vAdjustedArcs);
  assert(b);
  if (!b) {
    return false;
  }

  for (const auto& p : vAdjustedArcs) {
    const auto& arc = std::get<0>(p);
    //const auto& arcData  = std::get<1>(p);
    const bool isInsert = std::get<2>(p);
    if (isInsert) {
      _sArcs.emplace(arc);
    }
    else{
      const bool b = _sArcs.erase(arc);
      assert(b);
    }
  }
  
  assert(_topo.numSegs() == this->numArcs());
  return true;
}

bool Routable::removeFloatingRoutes()
{
  Vector<Tuple<Arc, ArcData, bool>> vAdjustedArcs;
  return removeFloatingRoutes(vAdjustedArcs);
}

bool Routable::removeFloatingRoutes(Vector<Tuple<Arc, ArcData, bool>>& vAdjustedArcs)
{
  _topo.removeFloating(vAdjustedArcs);
  for (const auto& p : vAdjustedArcs) {
    const auto& arc = std::get<0>(p);
    //const auto& arcData  = std::get<1>(p);
    const bool isInsert = std::get<2>(p);
    if (isInsert) {
      _sArcs.emplace(arc);
    }
    else{
      const bool b = _sArcs.erase(arc);
      assert(b);
    }
  }
  assert(_topo.numSegs() == this->numArcs());
  return true;
}

bool Routable::refineDanglingRoutes()
{
  Vector<Tuple<Arc, ArcData, bool>> vAdjustedArcs;
  return refineDanglingRoutes(vAdjustedArcs);
}

bool Routable::refineDanglingRoutes(Vector<Tuple<Arc, ArcData, bool>>& vAdjustedArcs){
  _topo.refineDangling(vAdjustedArcs, true, true);

  for (const auto& p : vAdjustedArcs) {
    const auto& arc = std::get<0>(p);
    //const auto& arcData  = std::get<1>(p);
    const bool isInsert = std::get<2>(p);
    if (isInsert) {
      _sArcs.emplace(arc);
    }
    else{
      const bool b = _sArcs.erase(arc);
      assert(b);
    }
  }
  
  assert(_topo.numSegs() == this->numArcs());
  return true;
}

void Net::init()
{
  for (Pin* pPin : _vpPins) {
    _topo.addPinPtr(pPin);
  }
  _topo.init();
  _topo.setNetPtr(this);

  //for (Routable& ro : _vRoutables) {
    //ro.init();
  //}
}

void Net::clearRouting()
{
  //_vArcs.clear();
  //_vpArcVias.clear();
  //_vArcViaOrients.clear();
  //_vArcWidthExts.clear();
  _mArcs.clear();
  //_vWires.clear();
  _sWires.clear();

  _topo.clear();

  init();

  //_vArcs.shrink_to_fit();
  //_vpArcVias.shrink_to_fit();
  //_vArcViaOrients.shrink_to_fit();
  //_vArcWidthExts.shrink_to_fit();
  //_vWires.shrink_to_fit();

  _isRouted = false;

  for (Routable& ro : _vRoutables) {
    ro.clearRouting();
  }
}

void Net::vArcs(Vector<Arc>& v) const
{
  for (const auto& arcEntry : _mArcs) {
    v.emplace_back(arcEntry.first);
  }
}

void Net::vArcs(Vector<Arc>& v, Vector<ArcData>& vd) const
{
  for (const auto& arcEntry : _mArcs) {
    v.emplace_back(arcEntry.first);
    vd.emplace_back(arcEntry.second);
  }
}

void Net::addRoute(const Arc& newArc, const ArcData& arcData)
{
  Vector<Tuple<Arc, ArcData, bool>> vAdjustedArcs;
  _topo.addRoute(newArc, arcData, vAdjustedArcs);

  for (const auto& p : vAdjustedArcs) {
    const auto& arc = std::get<0>(p);
    const auto& arcData = std::get<1>(p);
    const bool isInsert = std::get<2>(p);
    if (isInsert) {
      //if (arc.bVia()) {
        //if (arcData.pVia->minLayerIdx() != arc.zl()) {
          //std::cerr << newArc << std::endl << std::endl;;
          //for (const auto& p : vAdjustedArcs) {
            //std::cerr << std::get<0>(p) << " " << std::get<2>(p) << std::endl;
          //}
        //}
        //assert(arcData.pVia->minLayerIdx() == arc.zl());
      //}
      _mArcs.emplace(arc, arcData);
    }
    else {
      const bool b = _mArcs.erase(arc);
      assert(b);
    }
  }
  assert(_topo.numSegs() == this->numArcs());
}

bool Net::removeRoute(const Arc& arc)
{
  Vector<Tuple<Arc, ArcData, bool>> vAdjustedArcs;
  const bool b = _topo.removeRoute(arc, vAdjustedArcs);
  assert(b);
  if (!b) {
    return false;
  }

  for (const auto& p : vAdjustedArcs) {
    const auto& arc = std::get<0>(p);
    const auto& arcData = std::get<1>(p);
    const bool isInsert = std::get<2>(p);
    if (isInsert) {
      //if (arc.bVia()) {
        //assert(arcData.pVia->minLayerIdx() == arc.zl());
      //}
      _mArcs.emplace(arc, arcData);
    }
    else {
      const bool b = _mArcs.erase(arc);
      assert(b);
    }
  }

  // special handling
  auto getDim = [] (const Arc& s) -> Byte {
    if (s.p0().x() != s.p1().x()) return 0;
    if (s.p0().y() != s.p1().y()) return 1;
    if (s.p0().z() != s.p1().z()) return 2;
    assert(false);
  };
  const Int arcDim = getDim(arc);
  
  // handle routables
  for (Int i = 0; i < this->numRoutables(); ++i) {
    auto& ro = this->routable(i);
    if (ro.hasArc(arc)) {
      const bool b = ro.removeRoute(arc);
      //std::cerr << "remove arc" << std::endl;
      assert(b);
    }
    else {
      assert(!arc.bVia());

      Vector<Segment3d<Int>> vSegs;
      Vector<TopoTree::SegData> vSegData;
      ro.topo().querySegs(arc.p0(), arc.p1(), vSegs, vSegData);
      for (size_t j = 0; j < vSegs.size(); ++j) {
        const auto& s = vSegs.at(j);
        const auto& sData = vSegData.at(j);
        if (getDim(s) == arcDim) {
          if (arcDim == 0) { // x direction
            const Int y = s.p0().y();
            const Int z = s.p0().z();
            if (s.xl() <= arc.xl() and arc.xh() <= s.xh()) {
              ro.removeRoute(s);
              if (arc.xl() - s.xl() > 0) {
                const Segment3d<Int> newSeg(s.xl(), y, z, arc.xl(), y, z);
                ro.addRoute(newSeg, sData);
              }
              if (s.xh() - arc.xh() > 0) {
                const Segment3d<Int> newSeg(arc.xh(), y, z, s.xh(), y, z);
                ro.addRoute(newSeg, sData);
              }
            }
            else {
              //if (!(s.xh() <= arc.xl() or s.xl() >= arc.xh())) {
                //std::cerr << "s " << s << std::endl;
                //std::cerr << "arc " << arc << std::endl;
              //}
              assert(s.xh() <= arc.xl() or s.xl() >= arc.xh());
            }
          }
          else if (arcDim == 1) { // y direction
            const Int x = s.p0().x();
            const Int z = s.p0().z();
            if (s.yl() <= arc.yl() and arc.yh() <= s.yh()) {
              assert(s.yl() == arc.yl() or s.yh() == arc.yh());

              ro.removeRoute(s);
              if (arc.yl() - s.yl() > 0) {
                const Segment3d<Int> newSeg(x, s.yl(), z, x, arc.yl(), z);
                ro.addRoute(newSeg, sData);
              }
              if (s.yh() - arc.yh() > 0) {
                const Segment3d<Int> newSeg(x, arc.yh(), z, x, s.yh(), z);
                ro.addRoute(newSeg, sData);
              }
            }
            else {
              assert(s.yh() <= arc.yl() or s.yl() >= arc.yh());
            }

          }
        }
      }
    }
  }

  assert(_topo.numSegs() == this->numArcs());
  
  return true;
}

bool Net::removeFloatingRoutes()
{
  Vector<Tuple<Arc, ArcData, bool>> vAdjustedArcs;
  return removeFloatingRoutes(vAdjustedArcs);
}

bool Net::removeFloatingRoutes(Vector<Tuple<Arc, ArcData, bool>>& vAdjustedArcs)
{
  _topo.removeFloating(vAdjustedArcs);

  for (const auto& p : vAdjustedArcs) {
    const auto& arc = std::get<0>(p);
    const auto& arcData = std::get<1>(p);
    assert(arc.b90());
    assert(std::get<2>(p) == false);
    const bool b = _mArcs.erase(arc);
    assert(b);
    assert(arcData.roIdx < this->numRoutables());
    auto& ro = this->routable(arcData.roIdx);
    if (ro.hasArc(arc)) {
      // cannot use ro.removeRoute(merge) here. Otherwise, might leftover some
      // arcs
      const bool b0 = ro.topo().removeRoute(arc, false); // do not merge
      assert(b0);
      ro.arcs().erase(arc);
    }
  }
  assert(_topo.numSegs() == this->numArcs());
  return true;

}

bool Net::refineDanglingRoutes()
{
  Vector<Tuple<Arc, ArcData, bool>> vAdjustedArcs;
  return refineDanglingRoutes(vAdjustedArcs);
}

bool Net::refineDanglingRoutes(Vector<Tuple<Arc, ArcData, bool>>& vAdjustedArcs)
{
  _topo.refineDangling(vAdjustedArcs, true, true);

  for (const auto& p : vAdjustedArcs) {
    const auto& arc = std::get<0>(p);
    const auto& arcData = std::get<1>(p);
    const bool isInsert = std::get<2>(p);
    assert(arc.b90());
    if (isInsert) {
      assert(arcData.roIdx < this->numRoutables());

      //if (arc.bVia()) {
        //assert(arcData.pVia->minLayerIdx() == arc.zl());
      //}
      _mArcs.emplace(arc, arcData);

      auto& ro = this->routable(arcData.roIdx);
      ro.addRoute(arc, arcData);
    }
    else {
      const bool b = _mArcs.erase(arc);
      //if (!b) {
        ////return false;
        //std::cerr << arc << std::endl;
        //std::cerr << "marcs"<< std::endl;
        //for (const auto& e : _mArcs) {
          //std::cerr << e.first << std::endl;
        //}
        ////std::cerr << "removed"<<std::endl;
        ////for (const auto& p : vRemovedArcs) {
          ////std::cerr << p.first << std::endl;
        ////}
      //}
      assert(b);
      assert(arcData.roIdx < this->numRoutables());
      auto& ro = this->routable(arcData.roIdx);
      if (ro.hasArc(arc)) {
        const bool b0 = ro.removeRoute(arc);
        assert(b0);
      }

    }
  }
  //_topo.groupComps();
  //std::cerr << "net" << std::endl;
  //this->topo().showComps();
  //assert(this->numRoutables() == 1);
  //std::cerr << "ro" << std::endl;
  //auto& ro = this->routable(0);
  //ro.topo().groupComps(); 
  //ro.topo().showComps();

  //std::cerr << _topo.numSegs() << " " << ro.topo().numSegs() << std::endl;
  //assert(_topo.numComps() == ro.topo().numComps());
  //assert(_topo.numSegs() == ro.topo().numSegs());
  assert(_topo.numSegs() == this->numArcs());

  return true;
}

void Net::updateWireBbox()
{
  _wireBbox = _bbox;
  _minWireLayerIdx = _minPinLayerIdx;
  _maxWireLayerIdx = _maxPinLayerIdx;
  for (const auto& wire : _sWires) {
    const auto& box = wire.first;
    const Int layerIdx = wire.second;
    _wireBbox.set(std::min(_wireBbox.xl(), box.xl()),
                  std::min(_wireBbox.yl(), box.yl()),
                  std::max(_wireBbox.xh(), box.xh()),
                  std::max(_wireBbox.yh(), box.yh()));
    _minWireLayerIdx = std::min(_minWireLayerIdx, layerIdx);
    _maxWireLayerIdx = std::max(_maxWireLayerIdx, layerIdx);
  }
}

PROJECT_NAMESPACE_END
