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

#include "route/routeDrPS.hpp"
#include "drc/drcMgr.hpp"
#include "ds/array3d.hpp"


PROJECT_NAMESPACE_START

class RlUtils {
public:
  RlUtils() {}
  ~RlUtils() {}

  void                    genFeatsEdgesMasks(const CirDB& cir, const Net& net, const DrPsRouter& ps, const DrcMgr& drcMgr,
                                             Vector<Vector<Real>>& feats, Vector<Vector<Int>>& edges, Vector<Real>& masks,
                                             const Real normConstWL, const Real normConstHis) const;
  
  void                    genPathFeatsEdges(const CirDB& cir, const Net& net, 
                                            Vector<Vector<Real>>& pathFeats, Vector<Vector<Int>>& pathEdges,
                                            const Real normConstPathSegs,
                                            const Real normConstWL, const Real normConstVia, const Real normConstRes) const;

  void                    genFeatsEdgesMasksWithPathInfo(const CirDB& cir, const Net& net, const DrPsRouter& ps, const DrcMgr& drcMgr,
                                                         Vector<Vector<Real>>& segFeats, Vector<Vector<Real>>& pathFeats,
                                                         Vector<Vector<Int>>& seg2SegEdges, Vector<Vector<Int>>& path2SegEdges,
                                                         Vector<Real>& segMasks,
                                                         const Real normConstPathSegs,
                                                         const Real normConstWL, const Real normConstVia, const Real normConstRes,
                                                         const Real normConstHis) const;
  
  void                    genFeatsEdgesMasksWithSuperPathInfo(const CirDB& cir, const Net& net, const DrPsRouter& ps, const DrcMgr& drcMgr,
                                                              Vector<Vector<Real>>& segFeats, Vector<Vector<Real>>& pathFeats,
                                                              Vector<Vector<Int>>& seg2SegEdges, Vector<Vector<Int>>& path2SegEdges,
                                                              Vector<Real>& segMasks,
                                                              Vector<Vector<Real>>& superPathFeats, Vector<Vector<Int>>& path2SuperEdges,
                                                              const Real normConstPathSegs, 
                                                              const Real normConstWL, const Real normConstVia, const Real normConstRes,
                                                              const Real normConstHis) const;



  Pair<Real, Real>        getWlVia(const Net& net, const bool isScaled = false) const;
  Tuple<Real, Real, Int>  getWlViaDrv(const CirDB& cir, const Net& net, const DrPsRouter& ps, const DrcMgr& drcMgr, const bool isScaled = false) const;
  void                    addCurSol2History(const Net& net, DrPsRouter& ps, const Int cost) const;
  Pair<Real, Real>        getMinWlVia(Net& net, DrPsRouter& ps, const bool isScaled = false) const;

  Pair<Real, Real>        getLayersWlVia(const CirDB& cir, const Net& net, const DrPsRouter& ps, const DrcMgr& drcMgr,
                                         Vector<Real>& vWls, Vector<Real>& vVias, const bool isScaled = false) const;
  Tuple<Real, Real, Int>  getLayersWlViaDrv(const CirDB& cir, const Net& net, const DrPsRouter& ps, const DrcMgr& drcMgr,
                                            Vector<Real>& vWls, Vector<Real>& vVias, const bool isScaled = false) const;

  Vector<Real>            getPathRes(const CirDB& cir, const Net& net) const;

  void                    getSurroundingImage(const CirDB& cir, const Net& net, const DrPsRouter& ps,
                                              const Int xSize, const Int ySize,
                                              const Int xStep, const Int yStep,
                                              const Real expandFc, Vector<Vector<Vector<Vector<Real>>>>& image) const;

  void                    toVisGds(const CirDB& cir, const Net& net, const String& fileName) const;
  void                    toVisGds(const CirDB& cir, const Vector<Int>& vNetIds, const String& fileName) const;

  
  void                    genSuperPathFeatsEdgesImpl(const CirDB& cir, const Net& net, 
                                                     Vector<Vector<Real>>& superFeats,
                                                     Vector<Vector<Int>>& path2SuperEdges) const;

private:
  void                    genPathFeatsEdgesImpl(const CirDB& cir, const Net& net, const Vector<Segment3d<Int>>& vSegs, 
                                                Vector<Vector<Real>>& pathFeats, Vector<Vector<Int>>& pathEdges,
                                                const Real normConstPathSegs,
                                                const Real normConstWL, const Real normConstVia, const Real normConstRes) const;


  void                    genFeatsEdgesMasksImpl(const CirDB& cir, const Net& net, const Vector<Segment3d<Int>>& vSegs,
                                                 const DrPsRouter& ps, const DrcMgr& drcMgr,
                                                 Vector<Vector<Real>>& feats, Vector<Vector<Int>>& edges, Vector<Real>& masks,
                                                 const Real normConstWL, const Real normConstHis) const;
};



PROJECT_NAMESPACE_END
