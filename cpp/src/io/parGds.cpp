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
#include "parGds.hpp"
#include "geo/polygon2box.hpp"
#include "geo/box2polygon.hpp"
#include "geo/spatial.hpp"
#include "ds/disjointSet.hpp"

PROJECT_NAMESPACE_START

void GdsReader::parseRouting(const String& filename, const Vector<Int>& vNetIds)
{
  // Flatten Gds by the last cell
  GdsParser::GdsDB::GdsDB unflatenDB;
  GdsParser::GdsDB::GdsReader limboReader(unflatenDB);
  if (!limboReader(filename)) {
    spdlog::error("GdsReader::{} ERROR: Cannot open file {}!!\n", __func__, filename.c_str());
    exit(0);
  }
  
  
  // Flatten gds
  String topCellName = topCell(unflatenDB);
  auto flatCell = unflatenDB.extractCell(topCellName);
  //GdsParser::GdsDB::GdsDB gdsDB;
  //gdsDB.addCell(flatCell);
  
  // Build the mapping between the gds layer and the router layers
  buildLayerMap();
  
  // Process the shapes in gds
  for (const auto& obj : flatCell.objects()) {
    GdsParser::GdsDB::GdsObjectHelpers()(obj.first, obj.second, ExtractShapeLayerAction(_maskIdx2LayerIdx, _vPolygonLayers));
  }
	
  // scale the design
  const Real sc = 5e-10 / 1e-6 / _cir.physRes();

  if (std::fabs(sc - 1) > EPSILON) {
    for (auto& polygon : _vPolygonLayers) {
      polygon.scale(sc, sc);
    }
  }
  // save to db
  saveShapes(vNetIds);
}

/////////////////////////////////////////
//    Private functions                //
/////////////////////////////////////////
String GdsReader::topCell(const GdsParser::GdsDB::GdsDB& db) 
{
  // Whether each cell is found as the subcell of the other
  FlatHashMap<String, bool> mNameFound;
  // Iterate all the cells and record their names
  // Add reversely
  for (Int i = db.cells().size() - 1; i >= 0; --i) {
    mNameFound[db.cells().at(i).name()] = false;
  }
  for (const auto& cell : db.cells()) {
    for (const auto& obj : cell.objects()) {
      String name = "";
      GdsParser::GdsDB::GdsObjectHelpers()(obj.first, obj.second, GetSRefNameActionParser(name));
      if (name != "") {
        mNameFound[name] = true;
      }
    }
  }
  // Return the name that was not included in the other cells
  for (const auto& pair : mNameFound) {
    if (pair.first != "" and pair.second == false) {
      return pair.first;
    }
  }
  return "";
}

void GdsReader::buildLayerMap() 
{
  for (Int i = 0; i < _cir.numLayers(); ++i) {
    const auto& layer = _cir.layer(i);
    _maskIdx2LayerIdx.emplace(layer.gdsLayerDrawing(), layer.idx());
  }
}

void GdsReader::saveShapes(const Vector<Int>& vNetIds)
{

  // build tmp spatial shapes
  Vector<Vector<Pair<Box<Int>, Int>>> vLayerWireIds(_cir.numLayers());
  Vector<Pair<Box<Int>, Int>> vWires;
  for (const auto& poly : _vPolygonLayers) {
    const Int layerIdx = poly.layer;
    if (layerIdx != MAX_INT) { // ignore non routing layers
      Vector<Box<Int>> vBoxes;
      geo::polygon2Box(poly.pts, vBoxes);
      assert(vBoxes.size() == 1); 
      const auto& box = vBoxes.front();
      vLayerWireIds[layerIdx].emplace_back(box, vWires.size());
      vWires.emplace_back(box, layerIdx);
    }
  }
  Vector<SpatialMap<Int, Int>> vSpatialWireIds(_cir.numLayers());
  for (size_t i = 0; i < vSpatialWireIds.size(); ++i) {
    vSpatialWireIds[i] = SpatialMap<Int, Int>(vLayerWireIds[i]);
  }

  // group net shapes
  Vector<char> vis(vWires.size());

  DisjointSet ds(vWires.size());
  for (size_t i = 0; i < vWires.size(); ++i) {
    const auto& wire = vWires.at(i);
    const Int l = wire.second;
    const auto& layer = _cir.layer(l);
    if (layer.isMetal()) {
      const auto& b = wire.first;
      Vector<Int> vTouchedWireIds;
      vSpatialWireIds[l].query(b, vTouchedWireIds);
      for (const Int wireIdx : vTouchedWireIds) {
        ds.merge(i, wireIdx);
      }
    }
    else if (layer.isCut()) {
      const auto& b = wire.first;
      Vector<Int> vUpperWireIds, vLowerWireIds;
      vSpatialWireIds[l - 1].query(b, vLowerWireIds);
      vSpatialWireIds[l + 1].query(b, vUpperWireIds);
      for (const Int wireIdx : vLowerWireIds) {
        ds.merge(i, wireIdx);
      }
      for (const Int wireIdx : vUpperWireIds) {
        ds.merge(i, wireIdx);
      }
    }
  }

  // assign wire group to net idx
  auto touchNetIdx = [&] (const auto& wire) -> Int {
    Vector<Pin*> vpPins;
    _cir.spatialPins(wire.second).query(wire.first, vpPins);
    if (vpPins.empty()) {
      return -1;
    }
    const Int netIdx = vpPins.at(0)->net().idx();
//#ifndef NDEBUG
    //for (const auto pPin : vpPins) {
      //assert(pPin->net().idx() == netIdx);
    //}
//#endif
    return netIdx;
  };


  FlatHashMap<Int, Vector<Int>> rootIdx2WireIds;
  FlatHashMap<Int, Int> netIdx2RootIdx;
  for (size_t i = 0; i < vWires.size(); ++i) {
    const Int rootIdx = ds.find(i);
    if (rootIdx2WireIds.find(rootIdx) == rootIdx2WireIds.end()) {
      rootIdx2WireIds.emplace(rootIdx, Vector<Int>());
    }
    const auto& wire = vWires.at(i);
    rootIdx2WireIds[rootIdx].emplace_back(i);
    const Int netIdx = touchNetIdx(wire);
    if (netIdx > -1) {
      if (netIdx2RootIdx.find(netIdx) == netIdx2RootIdx.end()) {
        netIdx2RootIdx[netIdx] = rootIdx; 
      }
    }
  }
 
  FlatHashSet<Int> sNetIds;
  sNetIds.insert(vNetIds.begin(), vNetIds.end());

  for (const auto& item : netIdx2RootIdx) {
    const Int netIdx = item.first;
    if (sNetIds.empty() or sNetIds.find(netIdx) != sNetIds.end()) {
      const Int rootIdx = item.second;
      const auto& vWireIds = rootIdx2WireIds.at(rootIdx);
      Net& net = _cir.net(netIdx);
      net.setRouted(true);
      for (const Int wireIdx : vWireIds) {
        const auto& wire = vWires.at(wireIdx);
        net.sWires().emplace(wire);
        _cir.spatialNets(wire.second).insert(wire.first, &net);
      }
    }
  }


}

PROJECT_NAMESPACE_END
