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

#include <fstream>
#include <type_traits>

#include "parser.hpp"
#include "util/util.hpp"

PROJECT_NAMESPACE_START

void Parser::parse(const String& laygoConfig,
                   const String& techGrid,
                   const String& techMap,
                   const String& constraint,
                   const Vector<String>& vLefs,
                   const String& netlist,
                   const String& layerRules)
{

  spdlog::stopwatch sw;

#ifndef NDEBUG
  assert(util::fs::existFile(laygoConfig));
  assert(util::fs::existFile(techGrid));
  assert(util::fs::existFile(techMap));
  assert(util::fs::existFile(constraint));
  assert(util::fs::existFile(netlist));
  assert(util::fs::existFile(layerRules));
  for (size_t i = 0; i < vLefs.size(); ++i) {
    assert(util::fs::existFile(vLefs.at(i)));
  }
#endif

  YAML::Node cfg = YAML::LoadFile(laygoConfig);
  YAML::Node grd = YAML::LoadFile(techGrid);
  YAML::Node map = YAML::LoadFile(techMap);
  YAML::Node con = YAML::LoadFile(constraint);
  YAML::Node drc = YAML::LoadFile(layerRules);
  assert(cfg and cfg.IsMap());
  assert(grd and grd.IsMap());
  assert(map and map.IsSequence());
  assert(con and con.IsMap());
  assert(drc and drc.IsMap());

  /*********** physical resolution ***********/
  parsePhysRes(cfg);
  /*********** layers ************************/
  parseLayers(cfg, drc);
  /*********** prims+vias ********************/
  parseLef(vLefs);
  /*********** dev map ***********************/
  parseDevPrimMap(map);
  /*********** netlist ***********************/
  parseNetlist(netlist);

  /*********** constraints *******************/
  for (YAML::const_iterator it = con.begin(); it != con.end(); ++it) {
    const String& name = it->first.as<String>();
    const auto&   data = it->second;

    const YAML::Node& type = data["Type"];
    assert(type and type.IsScalar());

    const String& typeStr = type.as<String>();
    if (typeStr == "Design") {
      parseCstrDesign(name, data, grd);
    }
    else if (typeStr == "Region") {
      parseCstrRegion(name, data, grd);
    }
    else if (typeStr == "Assignment") {
      parseCstrNetAssignment(name, data, grd);
    }
    else if (typeStr == "Power") {
      parseCstrPower(name, data);
    }
    else if (typeStr == "Pin") {
      parseCstrPinAssignment(name, data, grd);
    }
    else if (typeStr == "Symmetry") {
      parseCstrPlaceSymmetry(name, data);
    }
    else if (typeStr == "Array") {
      parseCstrPlaceArray(name, data);
    }
    else if (typeStr == "Cluster") {
      parseCstrPlaceCluster(name, data);
    }
    else if (typeStr == "Extension") {
      parseCstrPlaceExtension(name, data);
    }
    else if (typeStr == "Preplace") {
      parseCstrPlacePreplace(name, data);
    }
    else if (typeStr == "Matching") {
      parseCstrRouteMatching(name, data, grd);
    }
    else if (typeStr == "PathMatching") {
      parseCstrRoutePathMatching(name, data, grd);
    }
    else if (typeStr == "Priority") {
      parseCstrNetPriority(name, data);
    }
    else if (typeStr == "Edge") {
      parseCstrPlaceEdgeDist(name, data);
    }
    else if (typeStr == "Order") {
      parseCstrPlaceOrder(name, data);
    }
    else if (typeStr == "Align") {
      parseCstrPlaceAlign(name, data);
    }
    else if (typeStr == "Disjoint") {
      parseCstrPlaceDisjoint(name, data);
    }
    else if (typeStr == "Row") {
      parseCstrPlaceRow(name, data);
    }
    else {
      //assert(false);
    }
  }
  // post-processing
  parseNetListPost();

  spdlog::info("{:<30} Elapsed: {:.3f} s", "Initialize database", sw);
}

void Parser::parsePhysRes(const YAML::Node& cfg)
{
  const YAML::Node& res = cfg["physical_resolution"];
  assert(res and res.IsScalar());
  _cir._physRes = res.as<Real>();
}

void Parser::parseLef(const Vector<String>& vFileNames)
{
  LefReader lefr(_cir);
  lefr.parse(vFileNames);
}

void Parser::parseLayers(const YAML::Node& cfg, const YAML::Node& drc)
{
  const YAML::Node& metalLayers = cfg["metal_layers"];
  const YAML::Node& cutLayers   = cfg["via_layers"];
  assert(metalLayers and metalLayers.IsSequence());
  assert(cutLayers and cutLayers.IsSequence());
  assert(metalLayers.size() == cutLayers.size() + 1);

  // vec: Vector<UniquePtr<T>>;  map: FlatHashMap<String, Int>
  auto addLayers = [&](const YAML::Node& data, auto& vec, auto& map) {
    for (const YAML::Node& n : data) {
      assert(n and n.IsSequence() and n.size() == 2);
      if (n[1].as<String>() == "drawing") {
        vec.emplace_back(new typename std::remove_pointer<typename std::remove_reference<decltype(vec)>::type::value_type>::type());
        auto& l    = *vec.back();
        l._selfIdx = vec.size() - 1;
        l._name    = n[0].as<String>();
        assert(map.find(l._name) == map.end());
        map.emplace(l._name, l._selfIdx);
      }
    }
  };
  addLayers(metalLayers, _cir._vpMetalLayers, _cir._mMetalLayerName2Idx);
  addLayers(cutLayers, _cir._vpCutLayers, _cir._mCutLayerName2Idx);

  // merge to _cir._vLayers
  auto& vpLayers       = _cir._vpLayers;
  auto& mLayerName2Idx = _cir._mLayerName2Idx;
  vpLayers.reserve(_cir._vpMetalLayers.size() + _cir._vpCutLayers.size());
  for (size_t i = 0; i < _cir._vpCutLayers.size(); ++i) {
    auto& plm  = _cir._vpMetalLayers[i];
    auto& plv  = _cir._vpCutLayers[i];
    plm->_idx = vpLayers.size();
    vpLayers.emplace_back(static_cast<Layer*>(plm));
    plv->_idx = vpLayers.size();
    vpLayers.emplace_back(static_cast<Layer*>(plv));
    mLayerName2Idx.emplace(plm->_name, plm->_idx);
    mLayerName2Idx.emplace(plv->_name, plv->_idx);
  }
  if (_cir._vpMetalLayers.size()) {
    auto& plm  = _cir._vpMetalLayers.back();
    plm->_idx = vpLayers.size();
    vpLayers.emplace_back(static_cast<Layer*>(plm));
    mLayerName2Idx.emplace(plm->_name, plm->_idx);
  }

  // parse drc rules
  for (size_t i = 0; i < _cir._vpMetalLayers.size(); ++i) {
    parseMetalLayerRules(drc, *_cir._vpMetalLayers.at(i));
  }
  for (size_t i = 0; i < _cir._vpCutLayers.size(); ++i) {
    parseCutLayerRules(drc, *_cir._vpCutLayers.at(i));
  }
}

void Parser::parseMetalLayerRules(const YAML::Node& drc, MetalLayer& layer)
{
  const YAML::Node& data = drc[layer.name()];
  if (!data) {
    spdlog::warn("[Parser] Layer {} drc not defined", layer.name());
    return;
  }

  const YAML::Node& type = data["Type"];
  assert(type and type.as<String>() == "Metal");

  const YAML::Node& gdsLayerNo = data["GdsLayerNo"];
  if (gdsLayerNo) {
    if (gdsLayerNo["drawing"]) {
      layer._gdsLayerDrawing = gdsLayerNo["drawing"].as<Int>();
    }
    if (gdsLayerNo["pin"]) {
      layer._gdsLayerPin = gdsLayerNo["pin"].as<Int>();
    }
    if (gdsLayerNo["colora"]) {
      layer._gdsLayerColora = gdsLayerNo["colora"].as<Int>();
    }
    if (gdsLayerNo["colorb"]) {
      layer._gdsLayerColorb = gdsLayerNo["colorb"].as<Int>();
    }
  }

  const YAML::Node& gdsDataType = data["GdsDatatype"];
  if (gdsDataType) {
    if (gdsDataType["drawing"]) {
      layer._gdsDataTypeDrawing = gdsDataType["drawing"].as<Int>();
    }
    if (gdsDataType["pin"]) {
      layer._gdsDataTypePin = gdsDataType["pin"].as<Int>();
    }
    if (gdsDataType["colora"]) {
      layer._gdsDataTypeColora = gdsDataType["colora"].as<Int>();
    }
    if (gdsDataType["colorb"]) {
      layer._gdsDataTypeColorb = gdsDataType["colorb"].as<Int>();
    }
  }

  if (data["Direction"]) {
    const String& dir = data["Direction"].as<String>();
    if (dir == "V") {
      layer._direction = RoutePrefE::ver;
    }
    else if (dir == "H") {
      layer._direction = RoutePrefE::hor;
    }
  }
  if (data["MinWidth"]) {
    layer._width = layer._minWidth = toDBUnit(data["MinWidth"].as<Real>());
  }  
  if (data["MinArea"]) {
    layer._area = toDBUnit2d(data["MinArea"].as<Real>());
  }
  if (data["ValidWidth"]) {
    for (const YAML::Node& n : data["ValidWidth"]) {
      layer._vValidWidths.emplace_back(toDBUnit(n.as<Real>()));
    }
  }
  if (data["ValidLength"]) {
    for (const YAML::Node& n : data["ValidLength"]) {
      layer._vValidLengths.emplace_back(toDBUnit(n.as<Real>()));
    }
  }
  if (data["EndOfLine"]) {
    const YAML::Node& eol = data["EndOfLine"];
    layer._vEolSpaces.emplace_back(toDBUnit(eol["Space"].as<Real>()));
    layer._vEolWidths.emplace_back(toDBUnit(eol["Width"].as<Real>()));
    layer._vEolWithins.emplace_back(toDBUnit(eol["Within"].as<Real>()));
  }
  if (data["SpacingTable"]) {
    const YAML::Node& spTable = data["SpacingTable"];
    for (const YAML::Node& n : spTable["PRL"]) {
      layer._vSpacingTablePrl.emplace_back(toDBUnit(n.as<Real>()));
    }
    const YAML::Node& widths = spTable["Widths"];
    for (const YAML::Node& width : widths) {
      layer._vSpacingTableWidth.emplace_back(toDBUnit(width["Width"].as<Real>()));
    }
    layer._spacingTable.resize(layer._vSpacingTableWidth.size(), layer._vSpacingTablePrl.size());
    for (size_t i = 0; i < widths.size(); ++i) {
      const YAML::Node& width = widths[i];
      const YAML::Node& spc = width["Space"];
      for (size_t j = 0; j < spc.size(); ++j) {
        layer._spacingTable.set(i, j, toDBUnit(spc[j].as<Real>()));
      }
    }
  }
  if (data["MinStep"]) {
    const YAML::Node& minStep = data["MinStep"];
    layer._minStep = toDBUnit(minStep["Step"].as<Real>());
    layer._maxEdges = minStep["MaxEdges"].as<Int>();
  }
  if (data["RCTable"]) {
    const YAML::Node& rcTable = data["RCTable"];
    for (const YAML::Node& rc : rcTable) {
      const Int width = toDBUnit(rc["Width"].as<Real>());
      
      const YAML::Node& r = rc["UnitR"];
      const Real r_L_3Sigma = r["L_3Sigma"] and !r["L_3Sigma"].IsNull() ? r["L_3Sigma"].as<Real>() * _cir._physRes : 0;
      const Real r_Mean     = r["Mean"]     and !r["Mean"].IsNull()     ? r["Mean"].as<Real>()     * _cir._physRes : 0;
      const Real r_U_3Sigma = r["U_3Sigma"] and !r["U_3Sigma"].IsNull() ? r["U_3Sigma"].as<Real>() * _cir._physRes : 0;
      layer._rTable[width] = std::make_tuple(r_L_3Sigma, r_Mean, r_U_3Sigma);

      const YAML::Node& cList = rc["UnitC"];
      for (const YAML::Node& e : cList) {
        const Int space = toDBUnit(e["Space"].as<Real>()); 

        const YAML::Node& c = e["C"];
        const Real c_L_3Sigma = c["L_3Sigma"] and !c["L_3Sigma"].IsNull() ? c["L_3Sigma"].as<Real>() * _cir._physRes : 0;
        const Real c_Mean     = c["Mean"]     and !c["Mean"].IsNull()     ? c["Mean"].as<Real>()     * _cir._physRes : 0;
        const Real c_U_3Sigma = c["U_3Sigma"] and !c["U_3Sigma"].IsNull() ? c["U_3Sigma"].as<Real>() * _cir._physRes : 0;
        
        const YAML::Node& cc = e["CC"];
        const Real cc_L_3Sigma = cc["L_3Sigma"] and !cc["L_3Sigma"].IsNull() ? cc["L_3Sigma"].as<Real>() * _cir._physRes : 0;
        const Real cc_Mean     = cc["Mean"]     and !cc["Mean"].IsNull()     ? cc["Mean"].as<Real>()     * _cir._physRes : 0;
        const Real cc_U_3Sigma = cc["U_3Sigma"] and !cc["U_3Sigma"].IsNull() ? cc["U_3Sigma"].as<Real>() * _cir._physRes : 0;

        layer._cTable[{width, space}] = std::make_pair(std::make_tuple(c_L_3Sigma, c_Mean, c_U_3Sigma),
                                                       std::make_tuple(cc_L_3Sigma, cc_Mean, cc_U_3Sigma));
      }

    }

  }

}

void Parser::parseCutLayerRules(const YAML::Node& drc, CutLayer& layer)
{
  const YAML::Node& data = drc[layer.name()];
  if (!data) {
    spdlog::warn("[Parser] Layer {} drc not defined", layer.name());
    return;
  }
  const YAML::Node& type = data["Type"];
  assert(type and type.as<String>() == "Via");

  const YAML::Node& gdsLayerNo = data["GdsLayerNo"];
  if (gdsLayerNo) {
    if (gdsLayerNo["drawing"]) {
      layer._gdsLayerDrawing = gdsLayerNo["drawing"].as<Int>();
    }
    if (gdsLayerNo["pin"]) {
      layer._gdsLayerPin = gdsLayerNo["pin"].as<Int>();
    }
    if (gdsLayerNo["colora"]) {
      layer._gdsLayerColora = gdsLayerNo["colora"].as<Int>();
    }
    if (gdsLayerNo["colorb"]) {
      layer._gdsLayerColora = gdsLayerNo["colorb"].as<Int>();
    }
  }

  const YAML::Node& gdsDataType = data["GdsDatatype"];
  if (gdsDataType) {
    if (gdsDataType["drawing"]) {
      layer._gdsDataTypeDrawing = gdsDataType["drawing"].as<Int>();
    }
    if (gdsDataType["pin"]) {
      layer._gdsDataTypePin = gdsDataType["pin"].as<Int>();
    }
    if (gdsDataType["colora"]) {
      layer._gdsDataTypeColora = gdsDataType["colora"].as<Int>();
    }
    if (gdsDataType["colorb"]) {
      layer._gdsDataTypeColorb = gdsDataType["colorb"].as<Int>();
    }
  }
  
  if (data["SpaceX"] and !data["SpaceX"].IsNull()) {
    layer._spaceX = toDBUnit(data["SpaceX"].as<Real>());
  }
  if (data["SpaceY"] and !data["SpaceY"].IsNull()) {
    layer._spaceY = toDBUnit(data["SpaceY"].as<Real>());
  }
  if (data["WidthX"] and !data["WidthX"].IsNull()) {
    layer._widthX = toDBUnit(data["WidthX"].as<Real>());
  }
  if (data["WidthY"] and !data["WidthY"].IsNull()) {
    layer._widthY = toDBUnit(data["WidthY"].as<Real>());
  }
  if (data["VencA_L"] and !data["VencA_L"].IsNull()) {
    layer._vencAL = toDBUnit(data["VencA_L"].as<Real>());
  }
  if (data["VencA_H"] and !data["VencA_H"].IsNull()) {
    layer._vencAH = toDBUnit(data["VencA_H"].as<Real>());
  }
  if (data["VencP_L"] and !data["VencP_L"].IsNull()) {
    layer._vencPL = toDBUnit(data["VencP_L"].as<Real>());
  }
  if (data["VencP_H"] and !data["VencP_L"].IsNull()) {
    layer._vencPH = toDBUnit(data["VencP_H"].as<Real>());
  }

  if (data["R"]) {
    const YAML::Node& r = data["R"];
    layer._rL3Sigma = r["L_3Sigma"] and !r["L_3Sigma"].IsNull() ? r["L_3Sigma"].as<Real>() : 0;
    layer._rMean    = r["Mean"]     and !r["Mean"].IsNull()     ? r["Mean"].as<Real>()     : 0;
    layer._rU3Sigma = r["U_3Sigma"] and !r["U_3Sigma"].IsNull() ? r["U_3Sigma"].as<Real>() : 0;
  }
  
}

void Parser::parseCstrDesign(const String& name, const YAML::Node& data, const YAML::Node& grd)
{
  assert(_cir._name == name);
  // default placement grid
  _cir._defaultPlaceGridIdx = parsePlacementGrid(data, grd);
  // default routing grid
  parseRoutingGrids(data, grd, false ,_cir._vDefaultRouteGridIds);
  //parseRoutingGrids(data, grd, true, _cir._vDefaultPowerRouteGridIds);
}

void Parser::parseCstrRegion(const String& name, const YAML::Node& data, const YAML::Node& grd)
{
  auto it = _cir._mRegionName2Idx.find(name); // name: region name
  if (it != _cir._mRegionName2Idx.end()) {
    assert(false);
  }
  const Int regIdx = _cir._vpRegions.size();
  _cir._vpRegions.emplace_back(new Region());
  _cir._mRegionName2Idx.emplace(name, regIdx);

  Region& reg = *_cir._vpRegions.back();
  //reg._pCir = &_cir;
  reg._name = name;
  reg._idx = regIdx;

  // place grid
  const Int placeGridIdx = parsePlacementGrid(data, grd);
  reg._pPlaceGrid        = _cir._vpPlaceGrids.at(placeGridIdx);

  // route grids
  Vector<Int> vRoutGridIds;
  parseRoutingGrids(data, grd, false, vRoutGridIds);
  for (const Int idx : vRoutGridIds) {
    reg._vpRouteGrids.emplace_back(_cir._vpRouteGrids.at(idx));
  }
  // power route grids
  //Vector<Int> vPowerRouteGridIds;
  //parseRoutingGrids(data, grd, true, vPowerRouteGridIds);
  //for (const Int idx : vPowerRouteGridIds) {
    //reg._vpPowerRouteGrids.emplace_back(_cir._vpPowerRouteGrids.at(idx).get());
  //}

  // cells
  for (const YAML::Node& n : data["Contains"]) {
    //const String& name = _cir._name + "/" + n.as<String>();
    const String& name = n.as<String>();
    auto it = _cir._mCellName2Idx.find(name);
    if (it != _cir._mCellName2Idx.end()) { // is cell
      Cell* pCell = _cir._vpCells.at(it->second);
      reg._vpCells.emplace_back(pCell);
      pCell->_pRegion = &reg;
    }
    else if (_cir._mBaseCellName2Ids.find(name) != _cir._mBaseCellName2Ids.end()) { // is base cell
      for (const Int cellIdx : _cir._mBaseCellName2Ids.at(name)) {
        Cell* pCell = _cir._vpCells.at(cellIdx);
        reg._vpCells.emplace_back(pCell);
        pCell->_pRegion = &reg;
      }
    }
    else { // is submodule
      for (const Int cellIdx : _cir._mSubmoduleName2Ids.at(name)) {
        Cell* pCell = _cir._vpCells.at(cellIdx);
        reg._vpCells.emplace_back(pCell);
        pCell->_pRegion = &reg;
      }
    }
  }
  // nets
  for (const Cell* pCell : reg._vpCells) {
    for (const Pin& pin : pCell->_vPins) {
      Net* pNet = pin._pNet;
      reg._vpNets.emplace_back(pNet);
      pNet->_pRegion = &reg;
    }
  }
  std::sort(reg._vpNets.begin(), reg._vpNets.end());
  reg._vpNets.resize(std::unique(reg._vpNets.begin(), reg._vpNets.end()) - reg._vpNets.begin());

  for (size_t i = 0; i < reg._vpNets.size(); ++i) {
    Net* pNet = reg._vpNets.at(i);

    // set default net assignment
    pNet->_vLayerValids.assign(_cir.numLayers(), true);
    pNet->_vLayerHorTracks.assign(_cir.numLayers(), 1);
    pNet->_vLayerVerTracks.assign(_cir.numLayers(), 1);
    pNet->_vLayerPriorities.assign(_cir.numLayers(), 1);
  }

  // edge cells
  parseEdgePrims(data, reg);
  // dummy cells
  parseDummyPrims(data, reg);
}

Int Parser::parsePlacementGrid(const YAML::Node& data, const YAML::Node& grd)
{
  const YAML::Node& pl = data["PlacementGrid"];
  assert(pl and pl.IsScalar());

  const String& gridStr    = pl.as<String>();

  auto it = _cir._mPlaceGridName2Idx.find(gridStr);
  if (it != _cir._mPlaceGridName2Idx.end())
    return it->second;

  const YAML::Node& gridSpec = grd[gridStr];
  assert(gridSpec and gridSpec.IsMap());

  const YAML::Node& type  = gridSpec["type"];
  const YAML::Node& xgrid = gridSpec["xgrid"];
  const YAML::Node& ygrid = gridSpec["ygrid"];
  const YAML::Node& xy0   = gridSpec["xy0"];
  const YAML::Node& xy1   = gridSpec["xy1"];
  assert(type and type.IsScalar() and type.as<String>() == "placement");
  assert(xgrid and xgrid.IsSequence() and xgrid.size() == 1);
  assert(ygrid and ygrid.IsSequence() and ygrid.size() == 1);
  assert(xy0 and xy0.IsSequence() and xy0.size() == 2);
  assert(xy1 and xy1.IsSequence() and xy1.size() == 2);

  const Int idx = _cir._vpPlaceGrids.size();
  _cir._mPlaceGridName2Idx.emplace(gridStr, idx);
  _cir._vpPlaceGrids.emplace_back(new PlaceGrid());
  PlaceGrid& pg = *_cir._vpPlaceGrids.back();

  pg._origin.setXY(toDBUnit(xgrid[0].as<Real>()), toDBUnit(ygrid[0].as<Real>()));
  pg._stepX = toDBUnit(xy1[0].as<Real>()) - toDBUnit(xy0[0].as<Real>());
  pg._stepY = toDBUnit(xy1[1].as<Real>()) - toDBUnit(xy0[1].as<Real>());
  pg._name  = gridStr;
  return idx;
}

void Parser::parseRoutingGrids(const YAML::Node& data, const YAML::Node& grd, const bool isPower,
                               Vector<Int>& vGridIds)
{
  auto& mRouteGridName2Idx = isPower ? _cir._mPowerRouteGridName2Idx : _cir._mRouteGridName2Idx;
  auto& vpRouteGrids = isPower ? _cir._vpPowerRouteGrids : _cir._vpRouteGrids;

  const YAML::Node& routeGrids = isPower ? data["PowerGrid"] : data["RoutingGrid"];
  assert(routeGrids and routeGrids.IsSequence());
  for (size_t i = 0; i < routeGrids.size(); ++i) {
    const String& gridStr = routeGrids[i].as<String>();
    auto it = mRouteGridName2Idx.find(gridStr);
    if (it != mRouteGridName2Idx.end()) {
      vGridIds.emplace_back(it->second);
      continue;
    }

    const Int idx = vpRouteGrids.size();
    vpRouteGrids.emplace_back(new RouteGrid());
    mRouteGridName2Idx.emplace(gridStr, idx);

    RouteGrid& rg = *vpRouteGrids.back();
    
    parseRoutingGrid(grd, gridStr, rg);

    vGridIds.emplace_back(idx);
  }
  std::sort(vGridIds.begin(), vGridIds.end());
  vGridIds.resize(std::unique(vGridIds.begin(), vGridIds.end()) - vGridIds.begin());
}

void Parser::parseRoutingGrid(const YAML::Node& grd, const String& gridName, RouteGrid& rg)
{
  const YAML::Node& gridSpec = grd[gridName];

  const YAML::Node& type    = gridSpec["type"];
  const YAML::Node& viamap  = gridSpec["viamap"];
  const YAML::Node& xgrid   = gridSpec["xgrid"];
  const YAML::Node& xy0     = gridSpec["xy0"];
  const YAML::Node& xy1     = gridSpec["xy1"];
  const YAML::Node& xlayer  = gridSpec["xlayer"];
  const YAML::Node& xwidth  = gridSpec["xwidth"];
  const YAML::Node& xconfig = gridSpec["xconfig"];
  const YAML::Node& ygrid   = gridSpec["ygrid"];
  const YAML::Node& ylayer  = gridSpec["ylayer"];
  const YAML::Node& ywidth  = gridSpec["ywidth"];
  const YAML::Node& yconfig = gridSpec["yconfig"];
  assert(type and type.as<String>() == "routing");
  assert(viamap and viamap.IsMap());
  assert(xgrid and xgrid.IsSequence());
  assert(xlayer and xlayer.IsSequence());
  assert(xwidth and xwidth.IsSequence());
  assert(xconfig and xconfig.IsSequence());
  assert(xy0 and xy0.IsSequence() and xy0.size() == 2);
  assert(xy1 and xy1.IsSequence() and xy1.size() == 2);
  assert(ygrid and ygrid.IsSequence());
  assert(ylayer and ylayer.IsSequence());
  assert(ywidth and ywidth.IsSequence());
  assert(yconfig and yconfig.IsSequence());
  
  rg._name = gridName;
  rg._xLayerIdx = _cir._mLayerName2Idx.at(xlayer[0].as<String>());
  rg._yLayerIdx = _cir._mLayerName2Idx.at(ylayer[0].as<String>());
  const Int x0 = toDBUnit(xy0[0].as<Real>()), x1 = toDBUnit(xy1[0].as<Real>());
  const Int y0 = toDBUnit(xy0[1].as<Real>()), y1 = toDBUnit(xy1[1].as<Real>());
  rg._origin.setXY(x0, y0);
  rg._stepX = x1 - x0;
  rg._stepY = y1 - y0;
  assert(xgrid.size() == xwidth.size());
  assert(xgrid.size() == xconfig.size());
  assert(ygrid.size() == ywidth.size());
  assert(ygrid.size() == yconfig.size());
  for (size_t j = 0; j < xgrid.size(); ++j) {
    rg._vXGrids.emplace_back(toDBUnit(xgrid[j].as<Real>()));
    rg._vXWidths.emplace_back(toDBUnit(xwidth[j].as<Real>()));
    const String& use = xconfig[j].as<String>();
    if (use == "P") {
      rg._vXUses.emplace_back(TrackUseE::power);
    }
    else if (use == "S") {
      rg._vXUses.emplace_back(TrackUseE::signal);
    }
    else if (use == "Z") {
      rg._vXUses.emplace_back(TrackUseE::free);
    }
    else {
      assert(false);
    }
  }
  for (size_t j = 0; j < ygrid.size(); ++j) {
    rg._vYGrids.emplace_back(toDBUnit(ygrid[j].as<Real>()));
    rg._vYWidths.emplace_back(toDBUnit(ywidth[j].as<Real>()));
    const String& use = yconfig[j].as<String>();
    if (use == "P") {
      rg._vYUses.emplace_back(TrackUseE::power);
    }
    else if (use == "S") {
      rg._vYUses.emplace_back(TrackUseE::signal);
    }
    else if (use == "Z") { 
      rg._vYUses.emplace_back(TrackUseE::free);
    }
    else {
      assert(false);
    }
  }

  rg._viaMap.resize(rg._vXGrids.size(), rg._vYGrids.size());
  for (YAML::const_iterator it = viamap.begin(); it != viamap.end(); ++it) {
    const String& viaName = it->first.as<String>();
    const Int viaIdx = _cir._mViaName2Idx.at(viaName);
    //if (rg._viaMap.find(viaName) == rg._viaMap.end()) {
      //Array2d<Byte>     valid(rg._vXGrids.size(), rg._vYGrids.size(), false);
      //const YAML::Node& locs = it->second;
      //for (size_t j = 0; j < locs.size(); ++j) {
        //assert(locs[j].size() == 2);
        //const Int x = locs[j][0].as<Int>();
        //const Int y = locs[j][1].as<Int>();
        //valid.set(x, y, true);
      //}
      //rg._viaMap.emplace(viaName, valid);
    //}
    //else {
      //assert(false);
    //}
    const Via* pVia = _cir.pVia(viaIdx);
    rg._vpVias.emplace_back(pVia);
    const YAML::Node& locs = it->second;
    for (size_t j = 0; j < locs.size(); ++j) {
      assert(locs[j].size() == 2);
      const Int x = locs[j][0].as<Int>();
      const Int y = locs[j][1].as<Int>();
      rg._viaMap.at(x, y).emplace_back(pVia);
    }

  }
}

void Parser::parseDevPrimMap(const YAML::Node& map)
{
  _cir._vpDevMaps.reserve(map.size());
  for (size_t i = 0; i < map.size(); ++i) {
    const YAML::Node& data = map[i];
    assert(data and data.IsMap());

    const YAML::Node& device   = data["Device"];
    const YAML::Node& prims    = data["Primitive"];
    const YAML::Node& abutment = data["Abutment"];
    const YAML::Node& devNets  = data["DeviceNets"];
    const YAML::Node& primNets = data["PrimitiveNets"];
    assert(device and device.IsScalar());
    assert(prims and prims.IsSequence());
    assert(abutment and abutment.IsScalar());
    assert(devNets and devNets.IsSequence());
    assert(primNets and primNets.IsSequence());

    const String& devName = device.as<String>();
    if (_cir._mDevName2Idx.find(devName) != _cir._mDevName2Idx.end()) {
      continue;
    }
    const Int devMapIdx = _cir._vpDevMaps.size();
    _cir._vpDevMaps.emplace_back(new DevMap());
    _cir._mDevName2Idx.emplace(devName, devMapIdx);
    DevMap& devMap    = *_cir._vpDevMaps.back();
    devMap._name      = devName;
    devMap._bAbutment = (abutment.as<String>() == "Sharing");
    
    // add prim to devMap
    assert(prims.size() == 1);
    for (size_t j = 0; j < prims.size(); ++j) {
      const YAML::Node& lib      = prims[j]["Library"];
      const YAML::Node& primType = prims[j]["PrimitiveType"];
      const YAML::Node& multi    = prims[j]["Multiplier"];
      assert(lib and lib.IsScalar());
      assert(primType and primType.IsScalar());
      assert(multi and multi.IsScalar());

      const String& libStr  = lib.as<String>();
      const String& primStr = primType.as<String>();
      const Int     multiplier = multi.as<Int>();

      auto it = _cir._mPrimName2Idx.find(primStr);
      if (it == _cir._mPrimName2Idx.end()) {
        spdlog::error("Unknown primitive in the mapping file: {}", primStr);
      }
      assert(it != _cir._mPrimName2Idx.end());
      devMap._pPrim = _cir._vpPrims.at(it->second);
      devMap._multiplier = multiplier;
    }
    // add devNet/primNet name mapping
    assert(devNets.size() == primNets.size());
    for (size_t j = 0; j < devNets.size(); ++j) {
      const String& devNet  = devNets[j].as<String>();
      const String& primNet = primNets[j].as<String>();
      devMap._mDevNetMap2Idx.emplace(devNet, devMap._vNetMaps.size());
      devMap._mPrimNetMap2Idx.emplace(primNet, devMap._vNetMaps.size());
      devMap._vNetMaps.emplace_back(devNet, primNet);
    }
  }
}


void Parser::parseEdgePrims(const YAML::Node& data,  Region& reg)
{

  const YAML::Node& edgeCells = data["EdgeCells"];
  const YAML::Node& east      = edgeCells["East"];
  const YAML::Node& west      = edgeCells["West"];
  const YAML::Node& south     = edgeCells["South"];
  const YAML::Node& north     = edgeCells["North"];
  const YAML::Node& northEast = edgeCells["NorthEast"];
  const YAML::Node& northWest = edgeCells["NorthWest"];
  const YAML::Node& southEast = edgeCells["SouthEast"];
  const YAML::Node& southWest = edgeCells["SouthWest"];

  auto addEdgePrim = [&] (const auto& dat) {
    if (dat) {
      const String& cellName  = dat["Cell"].template as<String>();
      const String& orientStr = dat["Orient"].template as<String>();

      Orient2dE ori = util::enumUtil::str2Val(Orient2dEStr2, orientStr);

      auto it = _cir._mPrimName2Idx.find(cellName);
      if (it != _cir._mPrimName2Idx.end()) {
        const Primitive* pPrim = _cir._vpPrims.at(it->second);
        reg._vpEdgePrims.emplace_back(pPrim);
        reg._vEdgePrimOrients.emplace_back(ori);
      }
      else {
        assert(false); // should already be parsed by LEF
      }
    }
  };
  addEdgePrim(east);
  addEdgePrim(west);
  addEdgePrim(south);
  addEdgePrim(north);
  addEdgePrim(northEast);
  addEdgePrim(northWest);
  addEdgePrim(southEast);
  addEdgePrim(southWest);
}

void Parser::parseDummyPrims(const YAML::Node& data, Region& reg)
{
  const YAML::Node& dummies    = data["DummyCells"];
  for (const YAML::Node& dummy : dummies) {
    const String& cellName = dummy.as<String>();

    auto it = _cir._mPrimName2Idx.find(cellName);
    if (it != _cir._mPrimName2Idx.end()) {
      const Primitive* pPrim = _cir._vpPrims.at(it->second);
      reg._vpDummyPrims.emplace_back(pPrim);
    }
    else {
      assert(false); // should already be parsed by LEF
    }
  }
}

void Parser::parseNetlist(const String& fileName)
{
  HspiceReader hsr;
  hsr.parse(fileName);

  const HspiceReader::Node& topNode = hsr.topNode();
  assert(topNode.instName() == topNode.tmplName());

  // circuit name
  _cir._name = topNode.tmplName();

  // flatten to primitives and add cells
  parseCells(hsr, topNode);

  // flatten and add nets
  parseNets(hsr, topNode);

}

void Parser::parseCells(const HspiceReader& hsr, const HspiceReader::Node& topNode)
{
  Stack<Tuple<String, String, const HspiceReader::Node*, const FlatHashMap<String, String>*>> st;
  //st.emplace(topNode.instName(), topNode.instName(), &topNode);
  st.emplace("", topNode.instName(), &topNode, nullptr);

  for (const auto& child : topNode.vChilds()) {
    if (_cir._mDevName2Idx.find(child.tmplName()) == _cir._mDevName2Idx.end()) {
      //_cir._mSubmoduleName2Ids.emplace(_cir._name + "/" + child.instName(), Vector<Int>());
      _cir._mSubmoduleName2Ids.emplace(child.instName(), Vector<Int>());
    }
  }

  while (st.size()) {
    const auto tup = st.top();
    st.pop();
    const String& name     = std::get<0>(tup);
    const String& instName = std::get<1>(tup);
    const auto&   node     = *std::get<2>(tup);
    const auto&   pmAttrs  = std::get<3>(tup);

    auto it = _cir._mDevName2Idx.find(node.tmplName());
    if (it != _cir._mDevName2Idx.end()) {
      const DevMap* pDevMap = _cir._vpDevMaps.at(it->second);

      // expand base cell
      Int numExpandCells = pDevMap->multiplier();
      if (pmAttrs) {
        if (pmAttrs->find("multi") != pmAttrs->end()) {
          numExpandCells *= stoi(pmAttrs->at("multi"));
        }
        if (pmAttrs->find("nf") != pmAttrs->end()) {
          numExpandCells *= stoi(pmAttrs->at("nf"));
        }
      }
      //std::cerr << name  << " " << numExpandCells << std::endl;
      for (Int i = 0; i < numExpandCells; ++i) {
        const String& cellName = numExpandCells > 1 ? name + "_" + std::to_string(i) : name;
        const String& cellInstName = numExpandCells > 1 ? instName + "_" + std::to_string(i) : instName;

        //std::cerr << cellName << std::endl;
        
        const Int cellIdx = _cir._vpCells.size();
        
        _cir._vpCells.emplace_back(new Cell());
        _cir._mCellName2Idx.emplace(cellName, cellIdx);

        Cell& cell     = *_cir._vpCells.back();
        //cell._pCir     = &_cir;
        cell._name     = cellName;
        cell._instName = cellInstName;
        cell._idx      = cellIdx;
        cell._pDevMap  = pDevMap;

        cell._vPins.reserve(node.numPorts());
        for (Int j = 0; j < node.numPorts(); ++j) {
          const String& devNetName = node.net(j);
          //const String& primNetName = pDevMap->devNet2PrimNet(devNetName);
          //spdlog::info("{} {} {}", cell.name(), devNetName, primNetName);
          //if (primNetName != "NC") {
          cell._vPins.emplace_back();
          Pin& pin = cell._vPins.back();
          pin._pCell = &cell;
          pin._name = cell._name + "/" + devNetName;
          pin._devNetName = devNetName;
          pin._idx = _cir._vpPins.size();

          _cir._mPinName2Idx.emplace(pin._name, _cir._vpPins.size());
          _cir._vpPins.emplace_back(&pin);
          //}
        }
        //spdlog::info("{}", node.tmplName());
        //spdlog::info("{} {} {} {}", cell.name(), cell.devMap().prim().name(), cell._vPins.size(), cell.devMap().prim().numPins());

        // set pin obs geometry
        cell.setPrim(cell.devMap().prim());

        // add submodule
        //size_t pos = cell._name.find_first_of("/", _cir._name.size() + 1, 1);
        size_t pos = cell._name.find_first_of("/", 0, 1);
        const String modName = cell._name.substr(0, pos);
        auto modIt = _cir._mSubmoduleName2Ids.find(modName);
        if (modIt != _cir._mSubmoduleName2Ids.end()) {
          modIt->second.emplace_back(cell._idx);
        }

        // add to base cell
        if (numExpandCells > 1) {
          auto baseIt = _cir._mBaseCellName2Ids.find(name);
          if (baseIt == _cir._mBaseCellName2Ids.end()) {
            baseIt = _cir._mBaseCellName2Ids.emplace(name, Vector<Int>()).first;
          }
          baseIt->second.emplace_back(cell._idx);
        }

      }

    }
    else {
      for (const auto& child : node.vChilds()) {
        const auto cn = &hsr.node(child.tmplName());
        st.emplace(name + (name == "" ? "" : "/") + child.instName(), child.instName(), cn, &child.mAttrs());
      }
    }
  }
  //spdlog::info("sdfsdfsd {}", _cir.numBaseCells());
  //for (const auto& a : _cir._mBaseCellName2Ids) {
    //spdlog::info("{}", a.first);
    //for (const Int idx : a.second) {
      //spdlog::info("{}", _cir._vpCells.at(idx)->name());
    //}
  //}
  //exit(0);
}

void Parser::parseNets(const HspiceReader& hsr, const HspiceReader::Node& topNode)
{
  auto buildNetWatchs = [&] (const HspiceReader::Node& node, Vector<Vector<Pair<Int, Int>>>& vvNetWatches) {
    vvNetWatches.resize(node.numNets());
    for (Int i = 0; i < node.numChilds(); ++i) {
      const auto& child = node.child(i);
      for (Int j = 0; j < child.numNets(); ++j) {
        const String& netName = child.net(j);
        vvNetWatches[node.netName2Idx(netName)].emplace_back(i, j);
      }
    }
  };

  std::function<void(const String&,
                     const HspiceReader::Node&,
                     const FlatHashMap<String, String>*,
                     const Int,
                     FlatHashMap<String, Vector<Vector<Pair<Int, Int>>>>&,
                     Net&)>
  flatten2TmplPins = [&] (const String& cellName,
                          const HspiceReader::Node& subCkt,
                          const FlatHashMap<String, String>* pmAttrs,
                          const Int port,
                          FlatHashMap<String, Vector<Vector<Pair<Int, Int>>>>& netWatches,
                          Net& net) {
    auto it = _cir._mDevName2Idx.find(subCkt.tmplName());
    if (it != _cir._mDevName2Idx.end()) {
      const DevMap& devMap = _cir.devMap(it->second);
      Int numExpandCells = devMap.multiplier();
      if (pmAttrs) {
        if (pmAttrs->find("multi") != pmAttrs->end()) {
          numExpandCells *= stoi(pmAttrs->at("multi"));
        }
        if (pmAttrs->find("nf") != pmAttrs->end()) {
          numExpandCells *= stoi(pmAttrs->at("nf"));
        }
      }
      for (Int i = 0; i < numExpandCells; ++i) {
        Cell& cell = numExpandCells > 1 ? _cir.cell(cellName + "_" + std::to_string(i)) : _cir.cell(cellName);
        Pin& pin = cell.pin(port);
        pin._pNet = &net;
        net._mPinName2Idx.emplace(pin.name(), net._vpPins.size());
        net._vpPins.emplace_back(&pin);
      }
    }
    else {
      if (netWatches.find(subCkt.tmplName()) == netWatches.end()) {
        netWatches.emplace(subCkt.tmplName(), Vector<Vector<Pair<Int, Int>>>());
        buildNetWatchs(subCkt, netWatches.at(subCkt.tmplName()));
      }
      const auto& watches = netWatches.at(subCkt.tmplName());

      for (const auto& pair : watches.at(port)) {
        const auto& child = subCkt.child(pair.first);
        const Int newPort = pair.second;
        flatten2TmplPins(cellName + (cellName == "" ? "" : "/") + child.instName(),
                         hsr.node(child.tmplName()),
                         &child.mAttrs(),
                         newPort,
                         netWatches,
                         net);
      }
    }
  };

 
  FlatHashMap<String, Vector<Vector<Pair<Int, Int>>>> netWatches;

  // add port nets from the top level
  for (Int i = 0; i < topNode.numPorts(); ++i) {
    const Int netIdx = _cir._vpNets.size();
    const String& netName = topNode.net(i);
    _cir._vpNets.emplace_back(new Net());
    _cir._mNetName2Idx.emplace(netName, netIdx);

    Net& net  = *_cir._vpNets.back();
    //net._pCir = &_cir;
    net._name = netName;
    net._idx  = netIdx;
    //if (_sVddNames.find(netName) != _sVddNames.end()) {
      //net._type = NetTypeE::vdd;
    //}
    //else if (_sVssNames.find(netName) != _sVssNames.end()) {
      //net._type = NetTypeE::vss;
    //}
    //flatten2TmplPins(topNode.instName(), topNode, i, netWatches, net);
    flatten2TmplPins("", topNode, nullptr, i, netWatches, net);
  }

  // flatten and add internel nets
  Stack<Tuple<String, const HspiceReader::Node*, const FlatHashMap<String, String>*>> st;
  //st.emplace(topNode.instName(), &topNode);
  st.emplace("", &topNode, nullptr);

  while (st.size()) {
    const auto tup = st.top();
    st.pop();

    const String& name    = std::get<0>(tup);
    const auto&   node    = *std::get<1>(tup);
    const auto&   pmAttrs = std::get<2>(tup);

    auto it = _cir._mDevName2Idx.find(node.tmplName());
    if (it == _cir._mDevName2Idx.end()) {
      if (node.numPorts() < node.numNets()) {
        for (Int i = node.numPorts(); i < node.numNets(); ++i) {
          const Int netIdx = _cir._vpNets.size();
          const String& postfix = node.net(i);
          const String netName = name + (name == "" ? "" : "/") + postfix;
          _cir._vpNets.emplace_back(new Net());
          _cir._mNetName2Idx.emplace(netName, netIdx);

          Net& net  = *_cir._vpNets.back();
          net._name = netName;
          net._idx  = netIdx;
          //if (_sVddNames.find(postfix) != _sVddNames.end()) {
            //net._type = NetTypeE::vdd;
          //}
          //else if (_sVssNames.find(postfix) != _sVssNames.end()) {
            //net._type = NetTypeE::vss;
          //}
          flatten2TmplPins(name, node, pmAttrs, i, netWatches, net);
        }
      }
      for (const auto& child : node.vChilds()) {
        st.emplace(name + (name == "" ? "" : "/") + child.instName(), &hsr.node(child.tmplName()), &child.mAttrs());
      }
    }
  }
}

void Parser::parseNetListPost()
{
  // set pin-net relation
  for (auto& pNet : _cir._vpNets) {
    for (Pin* pPin : pNet->_vpPins) {
      assert(pPin->_pNet == pNet);
      if (pNet->_type == NetTypeE::vdd) {
        pPin->_use = PinUseE::vdd;
      }
      else if (pNet->_type == NetTypeE::vss) {
        pPin->_use = PinUseE::vss;
      }
    }
  }

  // set pin-cell relation
  for (const auto& cellPtr : _cir._vpCells) {
    Cell& cell = *cellPtr;
    for (size_t i = 0; i < cell._vPins.size(); ++i) {
      Pin& pin = cell._vPins.at(i);
      //spdlog::info("{} {}", cell.name(), pin.name());
      assert(pin._pCell == &cell);
      if (pin.isUseVdd()) {
        cell._vddPinIdx = i;
      }
      else if (pin.isUseVss()) {
        cell._vssPinIdx = i;
      }
    }
    //assert(cell._vddPinIdx != -1);
    //assert(cell._vssPinIdx != -1);
  }
}

void Parser::parseCstrPlaceSymmetry(const String& name, const YAML::Node& data)
{
  const Int idx = _cir._vpPlaceSymCstrs.size();
  _cir._mPlaceSymCstrName2Idx.emplace(name, idx);
  _cir._vpPlaceSymCstrs.emplace_back(new PlaceSymCstr());

  PlaceSymCstr& sym = *_cir._vpPlaceSymCstrs.back();

  sym._name = name;
  sym._idx = idx;

  const String& axis = data["Axis"].as<String>();
  if (axis == "Vertical") {
    sym._axis = SymAxisE::ver;
  }
  else if (axis == "Horizontal") {
    sym._axis = SymAxisE::hor;
  }
  else if (axis == "Either") {
    sym._axis = SymAxisE::either;
  }
  else {
    assert(false);
  }
  
  const YAML::Node& contains = data["Contains"];
  for (const YAML::Node& n : contains) {
    const String& cn = n.as<String>();
    auto it = _cir._mCellName2Idx.find(cn);
    if (it != _cir._mCellName2Idx.end()) { // is cell
      Cell* pCell = _cir._vpCells.at(it->second);
      sym._mCellIdx2Idx.emplace(pCell->idx(), sym._vpCells.size());
      sym._vpCells.emplace_back(pCell);
      pCell->_vpPlaceSymCstrs.emplace_back(&sym);
    }
    else if (_cir._mBaseCellName2Ids.find(cn) != _cir._mBaseCellName2Ids.end()) { // is base cell
      for (const Int cellIdx : _cir._mBaseCellName2Ids.at(cn)) {
        Cell* pCell = _cir._vpCells.at(cellIdx);
        sym._mCellIdx2Idx.emplace(pCell->idx(), sym._vpCells.size());
        sym._vpCells.emplace_back(pCell);
        pCell->_vpPlaceSymCstrs.emplace_back(&sym);
      }
    }
    else if (_cir._mSubmoduleName2Ids.find(cn) != _cir._mSubmoduleName2Ids.end()) { // is submodule
      for (const Int cellIdx : _cir._mSubmoduleName2Ids.at(cn)) {
        Cell* pCell = _cir._vpCells.at(cellIdx);
        sym._mCellIdx2Idx.emplace(pCell->idx(), sym._vpCells.size());
        sym._vpCells.emplace_back(pCell);
        pCell->_vpPlaceSymCstrs.emplace_back(&sym);
      }
    }
    else {
      assert(false);
    }
  }

  const YAML::Node& partA = data["PartA"];
  const YAML::Node& partB = data["PartB"];
  assert(partA.size() == partB.size());
  sym._numPartAs = partA.size();
  sym._numSelfSyms = sym._vpCells.size() - 2 * partA.size();

  auto addSymPartE = [&] (const String& cn, const SymPartE p) { // is cell
    auto it = _cir._mCellName2Idx.find(cn);
    if (it != _cir._mCellName2Idx.end()) {
      const Cell& c = *_cir._vpCells.at(it->second);
      sym._vSymPartEs[sym._mCellIdx2Idx.at(c.idx())] = p;
    }
    else if (_cir._mBaseCellName2Ids.find(cn) != _cir._mBaseCellName2Ids.end()) { // is base cell
      for (const Int cellIdx : _cir._mBaseCellName2Ids.at(cn)) {
        const Cell& c = *_cir._vpCells.at(cellIdx);
        sym._vSymPartEs[sym._mCellIdx2Idx.at(c.idx())] = p;
      }
    }
    else if (_cir._mSubmoduleName2Ids.find(cn) != _cir._mSubmoduleName2Ids.end()) { // is submodule
      for (const Int cellIdx : _cir._mSubmoduleName2Ids.at(cn)) {
        const Cell& c = *_cir._vpCells.at(cellIdx);
        sym._vSymPartEs[sym._mCellIdx2Idx.at(c.idx())] = p;
      }
    }
    else {
      assert(false);
    }
  };
  auto setSymCells = [&] (const String& na, const String& nb) {
    auto ita = _cir._mCellName2Idx.find(na);
    auto itb = _cir._mCellName2Idx.find(nb);
    if (ita != _cir._mCellName2Idx.end() and itb != _cir._mCellName2Idx.end()) {
      const Cell& ca = *_cir._vpCells.at(ita->second);
      const Cell& cb = *_cir._vpCells.at(itb->second);
      auto caIdxIt = sym._mCellIdx2Idx.find(ca.idx());
      auto cbIdxIt = sym._mCellIdx2Idx.find(cb.idx());
      assert(caIdxIt != sym._mCellIdx2Idx.end());
      assert(cbIdxIt != sym._mCellIdx2Idx.end());
      sym._vSymCellIds[caIdxIt->second] = cbIdxIt->second;
      sym._vSymCellIds[cbIdxIt->second] = caIdxIt->second;
    }
    else if (_cir._mBaseCellName2Ids.find(na) != _cir._mBaseCellName2Ids.end() and
             _cir._mBaseCellName2Ids.find(nb) != _cir._mBaseCellName2Ids.end()) {
      const Vector<Int>& vAIds = _cir._mBaseCellName2Ids.at(na);
      const Vector<Int>& vBIds = _cir._mBaseCellName2Ids.at(nb);
      assert(vAIds.size() == vBIds.size());
      for (size_t i = 0; i < vAIds.size(); ++i) {
        const Cell& ca = *_cir._vpCells.at(vAIds.at(i));
        const Cell& cb = *_cir._vpCells.at(vBIds.at(i));
        auto caIdxIt = sym._mCellIdx2Idx.find(ca.idx());
        auto cbIdxIt = sym._mCellIdx2Idx.find(cb.idx());
        assert(caIdxIt != sym._mCellIdx2Idx.end());
        assert(cbIdxIt != sym._mCellIdx2Idx.end());
        sym._vSymCellIds[caIdxIt->second] = cbIdxIt->second;
        sym._vSymCellIds[cbIdxIt->second] = cbIdxIt->second;
      }

    }
    else if (_cir._mSubmoduleName2Ids.find(na) != _cir._mSubmoduleName2Ids.end() and
             _cir._mSubmoduleName2Ids.find(nb) != _cir._mSubmoduleName2Ids.end()) {
      const Vector<Int>& vAIds = _cir._mSubmoduleName2Ids.at(na);
      const Vector<Int>& vBIds = _cir._mSubmoduleName2Ids.at(nb);
      assert(vAIds.size() == vBIds.size());
      for (size_t i = 0; i < vAIds.size(); ++i) {
        const Cell& ca = *_cir._vpCells.at(vAIds.at(i));
        const Cell& cb = *_cir._vpCells.at(vBIds.at(i));
        auto caIdxIt = sym._mCellIdx2Idx.find(ca.idx());
        auto cbIdxIt = sym._mCellIdx2Idx.find(cb.idx());
        assert(caIdxIt != sym._mCellIdx2Idx.end());
        assert(cbIdxIt != sym._mCellIdx2Idx.end());
        sym._vSymCellIds[caIdxIt->second] = cbIdxIt->second;
        sym._vSymCellIds[cbIdxIt->second] = cbIdxIt->second;
      }
    }
    else {
      assert(false);
    }
  };

  sym._vSymPartEs.resize(sym._vpCells.size(), SymPartE::self);
  sym._vSymCellIds.resize(sym._vpCells.size(), -1);
  for (size_t i = 0; i < partA.size(); ++i) {
    const String& na = partA[i].as<String>();
    const String& nb = partB[i].as<String>();
    addSymPartE(na, SymPartE::a);
    addSymPartE(nb, SymPartE::b);
    setSymCells(na, nb);
  }

  assert(sym.check());
  
  Region& reg = *sym._vpCells.at(0)->_pRegion;
  reg._vpPlaceSymCstrs.emplace_back(&sym);
}

void Parser::parseCstrPlaceArray(const String& name, const YAML::Node& data)
{
  const Int idx = _cir._vpPlaceArrayCstrs.size();
  _cir._mPlaceArrayCstrName2Idx.emplace(name, idx);
  _cir._vpPlaceArrayCstrs.emplace_back(new PlaceArrayCstr());

  PlaceArrayCstr& arr = *_cir._vpPlaceArrayCstrs.back();

  arr._name = name;
  arr._idx = idx;

  if (data["Col"]) {
    const Int col = data["Col"].as<Int>();
    assert(col > 0);
    arr._col = col;
  }
  
  if (data["Row"]) {
    const Int row = data["Row"].as<Int>();
    assert(row > 0);
    arr._row = row;
  }

  arr._size = arr._col * arr._row;
 
  if (data["Pattern"]) {
    const String& pattern = data["Pattern"].as<String>();
    if (pattern == "ID") {
      arr._pattern = ArrayPatternE::id;
    }
    else if (pattern == "CC") {
      arr._pattern = ArrayPatternE::cc;
    }
    else if (pattern == "CS") {
      arr._pattern = ArrayPatternE::cs;
    }
    else {
      assert(pattern == "None");
    }
  }

  const YAML::Node& contains = data["Contains"];
  for (const YAML::Node& n : contains) {
    const String& cn = n.as<String>();
    auto it = _cir._mCellName2Idx.find(cn);
    if (it != _cir._mCellName2Idx.end()) { // is cell
      Cell* pCell = _cir._vpCells.at(it->second);
      arr._mCellIdx2Idx.emplace(pCell->idx(), arr._vpCells.size());
      arr._vpCells.emplace_back(pCell);
      pCell->_vpPlaceArrayCstrs.emplace_back(&arr);
    }
    else if (_cir._mBaseCellName2Ids.find(cn) != _cir._mBaseCellName2Ids.end()) { // is base cell
      for (const Int cellIdx : _cir._mBaseCellName2Ids.at(cn)) {
        Cell* pCell = _cir._vpCells.at(cellIdx);
        arr._mCellIdx2Idx.emplace(pCell->idx(), arr._vpCells.size());
        arr._vpCells.emplace_back(pCell);
        pCell->_vpPlaceArrayCstrs.emplace_back(&arr);
      }
    }
    else if (_cir._mSubmoduleName2Ids.find(cn) != _cir._mSubmoduleName2Ids.end()) { // is submodule
      for (const Int cellIdx : _cir._mSubmoduleName2Ids.at(cn)) {
        Cell* pCell = _cir._vpCells.at(cellIdx);
        arr._mCellIdx2Idx.emplace(pCell->idx(), arr._vpCells.size());
        arr._vpCells.emplace_back(pCell);
        pCell->_vpPlaceArrayCstrs.emplace_back(&arr);
      }
    }
    else if (_cir._mPlaceArrayCstrName2Idx.find(cn) != _cir._mPlaceArrayCstrName2Idx.end()) { // array of array
      PlaceArrayCstr* pArr = _cir.pPlaceArrayCstr(cn);
      arr._mArrayIdx2Idx.emplace(pArr->idx(), arr._vpArrays.size());
      arr._vpArrays.emplace_back(pArr);
    }
    else {
      assert(false);
    }
  }
  
  auto addArrayPartE = [&] (const String& cn, const ArrayPartE p) {
    auto it = _cir._mCellName2Idx.find(cn);
    if (it != _cir._mCellName2Idx.end()) { // is cell
      const Cell& c = *_cir._vpCells.at(it->second);
      arr._vArrayPartEs[arr._mCellIdx2Idx.at(c.idx())] = p;
    }
    else if (_cir._mBaseCellName2Ids.find(cn) != _cir._mBaseCellName2Ids.end()) { // base cell
      for (const Int cellIdx : _cir._mBaseCellName2Ids.at(cn)) {
        const Cell& c = *_cir._vpCells.at(cellIdx);
        arr._vArrayPartEs[arr._mCellIdx2Idx.at(c.idx())] = p;
      }
    }
    else if (_cir._mSubmoduleName2Ids.find(cn) != _cir._mSubmoduleName2Ids.end()) { // submodule 
      for (const Int cellIdx : _cir._mSubmoduleName2Ids.at(cn)) {
        const Cell& c = *_cir._vpCells.at(cellIdx);
        arr._vArrayPartEs[arr._mCellIdx2Idx.at(c.idx())] = p;
      }
    }
    else if (_cir._mPlaceArrayCstrName2Idx.find(cn) != _cir._mPlaceArrayCstrName2Idx.end()) { // array of array
      const PlaceArrayCstr& ac = *_cir.pPlaceArrayCstr(cn);
      arr._vArrayPartEs[arr._mArrayIdx2Idx.at(ac.idx())] = p;
    }

  };

  const YAML::Node& partA = data["PartA"];
  const YAML::Node& partB = data["PartB"];
 
  arr._vArrayPartEs.resize(arr._vpCells.size(), ArrayPartE::undef);
  if (partA) {
    for (size_t i = 0; i < partA.size(); ++i) {
      const String& n = partA[i].as<String>();
      addArrayPartE(n, ArrayPartE::a);
    }
  }
  if (partB) {
    for (size_t i = 0; i < partB.size(); ++i) {
      const String& n = partB[i].as<String>();
      addArrayPartE(n, ArrayPartE::b);
    }
  }

  Region& reg = arr._vpCells.empty() ? const_cast<Region&>(arr._vpArrays.at(0)->region()) : *arr._vpCells.at(0)->_pRegion;
  reg._vpPlaceArrayCstrs.emplace_back(&arr);
}

void Parser::parseCstrPlaceCluster(const String& name, const YAML::Node& data)
{
  const Int idx = _cir._vpPlaceClusterCstrs.size();
  _cir._mPlaceClusterCstrName2Idx.emplace(name, idx);
  _cir._vpPlaceClusterCstrs.emplace_back(new PlaceClusterCstr());

  PlaceClusterCstr& clus = *_cir._vpPlaceClusterCstrs.back();

  clus._name = name;
  clus._idx = idx;

  const YAML::Node& contains = data["Contains"];
  for (const YAML::Node& n : contains) {
    const String& cellName = n.as<String>();
    auto it = _cir._mCellName2Idx.find(cellName);
    if (it != _cir._mCellName2Idx.end()) { // is cell
      Cell* pCell = _cir._vpCells.at(it->second);
      clus._vpCells.emplace_back(pCell);
      pCell->_vpPlaceClusterCstrs.emplace_back(&clus);
    }
    else if (_cir._mBaseCellName2Ids.find(cellName) != _cir._mBaseCellName2Ids.end()) { // is base cell
      for (const Int cellIdx : _cir._mBaseCellName2Ids.at(cellName)) {
        Cell* pCell = _cir._vpCells.at(cellIdx);
        clus._vpCells.emplace_back(pCell);
        pCell->_vpPlaceClusterCstrs.emplace_back(&clus);
      }
    }
    else { // is submodule
      for (const Int cellIdx : _cir._mSubmoduleName2Ids.at(cellName)) {
        Cell* pCell = _cir._vpCells.at(cellIdx);
        clus._vpCells.emplace_back(pCell);
        pCell->_vpPlaceClusterCstrs.emplace_back(&clus);
      }
    }
  }

  const Int weight = data["Weight"].as<Int>();
  clus._weight = weight;
}

void Parser::parseCstrPlacePreplace(const String& name, const YAML::Node& data)
{
  const Int idx = _cir._vpPrePlaceCstrs.size();
  _cir._mPrePlaceCstrName2Idx.emplace(name, idx);
  _cir._vpPrePlaceCstrs.emplace_back(new PrePlaceCstr());

  PrePlaceCstr& pre = *_cir._vpPrePlaceCstrs.back();

  pre._name = name;
  pre._idx = idx;

  const YAML::Node& contains = data["Contains"];
  const YAML::Node& locs = data["Locations"];
  const YAML::Node& orients = data["Orients"];

  assert(contains.size() == locs.size());
  assert(contains.size() == orients.size());

  for (size_t i = 0; i < contains.size(); ++i) {
    const YAML::Node& n = contains[i];
    const YAML::Node& loc = locs[i];
    const YAML::Node& ori = orients[i];
    assert(loc.size() == 2);

    const String& name = n.as<String>();
    const String& oriStr = ori.as<String>();

    if (_cir._mCellName2Idx.find(name) != _cir._mCellName2Idx.end()) {
      const Int idx = _cir._mCellName2Idx.at(name);
      Cell* pCell = _cir._vpCells.at(idx);
      pre._vpCells.emplace_back(pCell);
      const Int x = toDBUnit(loc[0].as<Real>());
      const Int y = toDBUnit(loc[1].as<Real>());
      pre._vCellLocs.emplace_back(x, y);
      
      Orient2dE orient = Orient2dE::undef;
      if (oriStr == "n" or oriStr == "R0") {
        orient = Orient2dE::n;
      }
      else if (oriStr == "w" or oriStr == "R90") {
        orient = Orient2dE::w;
      }
      else if (oriStr == "s" or oriStr == "R180") {
        orient = Orient2dE::s;
      }
      else if (oriStr == "e" or oriStr == "R270") {
        orient = Orient2dE::e;
      }
      else if (oriStr == "fn" or oriStr == "MY") {
        orient = Orient2dE::fn;
      }
      else if (oriStr == "fw" or oriStr == "MX90") {
        orient = Orient2dE::fw;
      }
      else if (oriStr == "fs" or oriStr == "MX") {
        orient = Orient2dE::fs;
      }
      else if (oriStr == "fe" or oriStr == "MY90") {
        orient = Orient2dE::fe;
      }
      else {
        assert(false);
      }
      pre._vCellOrients.emplace_back(orient);

      pCell->_pPrePlaceCstr = &pre;
    }
    else if (_cir._mRegionName2Idx.find(name) != _cir._mRegionName2Idx.end()) {
      const Int idx = _cir._mRegionName2Idx.at(name);
      Region* pReg = _cir._vpRegions.at(idx);
      pre._vpRegions.emplace_back(pReg);
      const Int x = toDBUnit(loc[0].as<Real>());
      const Int y = toDBUnit(loc[1].as<Real>());
      pre._vRegLocs.emplace_back(x, y);
      //pReg->_vpPrePlaceCstrs.emplace_back(&pre);
    }
  }

  //Region& reg = *pre._vpCells.at(0)->_pRegion;
  //reg._vpPrePlaceCstrs.emplace_back(&pre);
}

void Parser::parseCstrNetAssignment(const String& name, const YAML::Node& data, const YAML::Node& grd)
{
  Vector<Byte> vLayerValids(_cir.numLayers(), false);
  Vector<Int>  vLayerHorTracks(_cir.numLayers(), 1);
  Vector<Int>  vLayerVerTracks(_cir.numLayers(), 1);
  Vector<Int>  vLayerPriorities(_cir.numLayers(), 1);


  const YAML::Node& assigns = data["Assignment"];
  for (const YAML::Node& assign : assigns) {
    const String& gridName  = assign["Grid"].as<String>();
    const Int     horTracks = assign["HTracks"].as<Int>();
    const Int     verTracks = assign["VTracks"].as<Int>();
    const Int     priority  = assign["Priority"].as<Int>();
    if (_cir._mRouteGridName2Idx.find(gridName) == _cir._mRouteGridName2Idx.end()) {
      const Int idx = _cir._vpRouteGrids.size();
      _cir._vpRouteGrids.emplace_back(new RouteGrid());
      _cir._mRouteGridName2Idx.emplace(gridName, idx);
      RouteGrid& rg = *_cir._vpRouteGrids.back();
      parseRoutingGrid(grd, gridName, rg);
    }
    const RouteGrid& rg = _cir.routeGrid(gridName);
    
    vLayerValids[rg.xLayerIdx()] = true;
    vLayerValids[rg.yLayerIdx()] = true;
    vLayerVerTracks[rg.xLayerIdx()] = verTracks;
    vLayerHorTracks[rg.yLayerIdx()] = horTracks;
    vLayerPriorities[rg.xLayerIdx()] = priority;
    vLayerPriorities[rg.yLayerIdx()] = priority;
  }

  const YAML::Node& contains = data["Contains"];
  for (const YAML::Node& contain : contains) {
    const String& netName = contain.as<String>();
    Net& net = _cir.net(netName);
    net._vLayerValids = vLayerValids;
    net._vLayerHorTracks = vLayerHorTracks;
    net._vLayerVerTracks = vLayerVerTracks;
    net._vLayerPriorities = vLayerPriorities;
  }
}

void Parser::parseCstrPower(const String& name, const YAML::Node& data)
{
  const YAML::Node& nodes = data["Assignment"];
  for (const YAML::Node& node : nodes) {
    const String& regName = node["Region"].as<String>();
    Region& reg = *_cir._vpRegions.at(_cir._mRegionName2Idx.at(regName));

    const YAML::Node& vddNets = node["Vdd"];
    for (const YAML::Node& net : vddNets) {
      const String& netName = net.as<String>();
      Net* pNet = _cir.pNet(netName);
      pNet->_type = NetTypeE::vdd;
      reg._vpPowerNets.emplace_back(pNet);
    }

    const YAML::Node& vssNets = node["Vss"];
    for (const YAML::Node& net : vssNets) {
      const String& netName = net.as<String>();
      Net* pNet = _cir.pNet(netName);
      pNet->_type = NetTypeE::vss;
      reg._vpPowerNets.emplace_back(pNet);

    }
  }
}

void Parser::parseCstrPinAssignment(const String& name, const YAML::Node& data, const YAML::Node& grd)
{
  const YAML::Node& nodes = data["Assignment"];

  for (const YAML::Node& node : nodes) {
    const String& locStr = node["Location"].as<String>();
    const String& gridName = node["Grid"].as<String>();
    if (_cir._mRouteGridName2Idx.find(gridName) == _cir._mRouteGridName2Idx.end()) {
      const Int idx = _cir._vpRouteGrids.size();
      _cir._vpRouteGrids.emplace_back(new RouteGrid());
      _cir._mRouteGridName2Idx.emplace(gridName, idx);
      RouteGrid& rg = *_cir._vpRouteGrids.back();
      parseRoutingGrid(grd, gridName, rg);
    }
    const YAML::Node& pins = node["Pins"];
    for (const YAML::Node& pin : pins) {
      const String& pinName = pin.as<String>();

      Net& net = _cir.net(pinName);
      net._ioLoc = util::enumUtil::str2Val(NetIOLocEStr, locStr);
      net._pIORouteGrid = _cir.pRouteGrid(gridName);
      
      const Int idx = _cir._vpIOPins.size();
      _cir._vpIOPins.emplace_back(new Pin());
      _cir._mIOPinName2Idx.emplace(pinName, idx);

      const IOPinLocE ioPinLoc = util::enumUtil::str2Val(IOPinLocEStr, locStr);
      _cir._vIOPinLocs.emplace_back(ioPinLoc);
      _cir._vIOPinBboxes.emplace_back();

      Pin& ioPin = *_cir._vpIOPins.back();
      ioPin._name = pinName;
      ioPin._pNet = &net;
      ioPin._dir = PinDirE::inout;
      ioPin._isIO = true;
      ioPin._ioIdx = idx;
      assert(ioPin._pNet->_pIORouteGrid != nullptr);

      switch (ioPinLoc) {
        case IOPinLocE::left:
          _cir._vLeftIOPinIds.emplace_back(ioPin.ioIdx());
          break;
        case IOPinLocE::right:
          _cir._vRightIOPinIds.emplace_back(ioPin.ioIdx());
          break;
        case IOPinLocE::bottom:
          _cir._vBottomIOPinIds.emplace_back(ioPin.ioIdx());
          break;
        case IOPinLocE::top:
          _cir._vTopIOPinIds.emplace_back(ioPin.ioIdx());
          break;
        default:
          assert(false);
      }

      net._vpPins.emplace_back(&ioPin);
    }
  }

}

void Parser::parseCstrPlaceExtension(const String& name, const YAML::Node& data) 
{
  const Int idx = _cir._vpPlaceExtCstrs.size();
  _cir._mPlaceExtCstrName2Idx.emplace(name, idx);
  _cir._vpPlaceExtCstrs.emplace_back(new PlaceExtCstr());

  PlaceExtCstr& ext = *_cir._vpPlaceExtCstrs.back();

  ext._name = name;
  ext._idx = idx;

  const YAML::Node& left = data["Left"];
  const YAML::Node& right = data["Right"];
  const YAML::Node& bottom = data["Bottom"];
  const YAML::Node& top = data["Top"];

  const String& lCellName = left["Cell"].as<String>();
  const String& rCellName = right["Cell"].as<String>();
  const String& bCellName = top["Cell"].as<String>();
  const String& tCellName = bottom["Cell"].as<String>();

  const Primitive* pPrimL = lCellName != "NA" ? _cir.pPrim(lCellName) : nullptr;
  const Primitive* pPrimR = rCellName != "NA" ? _cir.pPrim(rCellName) : nullptr;
  const Primitive* pPrimB = bCellName != "NA" ? _cir.pPrim(bCellName) : nullptr;
  const Primitive* pPrimT = tCellName != "NA" ? _cir.pPrim(tCellName) : nullptr;

  const Int gL = left["Grid"].as<Int>();
  const Int gR = right["Grid"].as<Int>();
  const Int gB = bottom["Grid"].as<Int>();
  const Int gT = top["Grid"].as<Int>();
  
  ext._left = std::make_pair(pPrimL, gL);
  ext._right = std::make_pair(pPrimR, gR);
  ext._bottom = std::make_pair(pPrimB, gB);
  ext._top = std::make_pair(pPrimT, gT);

  Region* pReg = nullptr;
  const YAML::Node& contains = data["Contains"];
  for (const YAML::Node& n : contains) {
    const String& name = n.as<String>();
    if (_cir._mCellName2Idx.find(name) != _cir._mCellName2Idx.end()) {
      Cell* pCell = _cir.pCell(name);
      ext._vpCells.emplace_back(pCell);
      if (ext._type == ExtTypeE::undef) {
        ext._type = ExtTypeE::cell;
        pReg = pCell->pRegion();
      }
      else {
        assert(ext._type == ExtTypeE::cell);
        assert(pReg == pCell->pRegion());
      }
      pCell->_pPlaceExtCstr = &ext;
    }
    else if (_cir._mRegionName2Idx.find(name) != _cir._mRegionName2Idx.end()) {
      Region* pR = _cir.pRegion(name);
      ext._vpRegions.emplace_back(pR);
      if (ext._type == ExtTypeE::undef) {
        ext._type = ExtTypeE::region;
        pReg = pR;
      }
      else {
        assert(ext._type == ExtTypeE::region);
        assert(pReg == pR);
      }
    }
    else if (_cir._mPlaceArrayCstrName2Idx.find(name) != _cir._mPlaceArrayCstrName2Idx.end()) {
      PlaceArrayCstr* pArr = _cir.pPlaceArrayCstr(name);
      ext._vpPlaceArrayCstrs.emplace_back(pArr);
      if (ext._type == ExtTypeE::undef) {
        ext._type = ExtTypeE::array;
        pReg = const_cast<Region*>(pArr->pRegion());
      }
      else {
        assert(ext._type == ExtTypeE::array);
        assert(pReg == pArr->pRegion());
      }
    }
    else {
      assert(false);
    }
  }
  pReg->_vpPlaceExtCstrs.emplace_back(&ext);
}

void Parser::parseCstrPlaceEdgeDist(const String& name, const YAML::Node& data)
{
  const Int idx = _cir._vpPlaceEdgeDistCstrs.size();
  _cir._mPlaceEdgeDistCstrName2Idx.emplace(name, idx);
  _cir._vpPlaceEdgeDistCstrs.emplace_back(new PlaceEdgeDistCstr());

  PlaceEdgeDistCstr& ed = *_cir._vpPlaceEdgeDistCstrs.back();

  ed._name = name;
  ed._idx = idx;

  const YAML::Node& dist = data["Distance"];
  assert(dist.size() == 4);

  ed._distL = dist[0].as<Int>();
  ed._distR = dist[1].as<Int>();
  ed._distB = dist[2].as<Int>();
  ed._distT = dist[3].as<Int>();

  const YAML::Node& contains = data["Contains"];
  for (const YAML::Node& n : contains) {
    const String& cn = n.as<String>();
    if (_cir._mCellName2Idx.find(cn) != _cir._mCellName2Idx.end()) {
      Cell& c = _cir.cell(cn);
      ed._vpCells.emplace_back(&c);
      c._pPlaceEdgeDistCstr = &ed;
    }
    else if (_cir._mBaseCellName2Ids.find(cn) != _cir._mBaseCellName2Ids.end()) {
      for (const Int cellIdx : _cir._mBaseCellName2Ids.at(cn)) {
        Cell& c = _cir.cell(cellIdx);
        ed._vpCells.emplace_back(&c);
        c._pPlaceEdgeDistCstr = &ed;
      }
    }
    else if (_cir._mSubmoduleName2Ids.find(cn) != _cir._mSubmoduleName2Ids.end()) {
      for (const Int cellIdx : _cir._mSubmoduleName2Ids.at(cn)) {
        Cell& c = _cir.cell(cellIdx);
        ed._vpCells.emplace_back(&c);
        c._pPlaceEdgeDistCstr = &ed;
      }
    }
  }
  Region& reg = ed._vpCells.at(0)->region();
  reg._vpPlaceEdgeDistCstrs.emplace_back(&ed);

}

void Parser::parseCstrPlaceOrder(const String& name, const YAML::Node& data)
{
  const Int idx = _cir._vpPlaceOrderCstrs.size();
  _cir._mPlaceOrderCstrName2Idx.emplace(name, idx);
  _cir._vpPlaceOrderCstrs.emplace_back(new PlaceOrderCstr());

  PlaceOrderCstr& order = *_cir._vpPlaceOrderCstrs.back();

  order._name = name;
  order._idx = idx;

  const String& dirStr = data["Direction"].as<String>();

  if (dirStr == "Left") {
    order._dir = Direction2dE::left;
  }
  else if (dirStr == "Right") {
    order._dir = Direction2dE::right;
  }
  else if (dirStr == "Down") {
    order._dir = Direction2dE::down;
  }
  else if (dirStr == "Up") {
    order._dir = Direction2dE::up;
  }
  else {
    assert(false);
  }

  const String& sub = data["SubType"].as<String>();
  if (sub == "Strict") {
    order._isStrict = true;
  }
  else if (sub == "Weak") {
    order._isStrict = false;
  }
  else {
    assert(false);
  }

  const YAML::Node& contains = data["Contains"];
  for (const YAML::Node& n : contains) {
    order._vvpCells.emplace_back();
    const String& cn = n.as<String>();
    if (_cir._mCellName2Idx.find(cn) != _cir._mCellName2Idx.end()) { // is cell
      Cell& c = _cir.cell(cn);
      order._vvpCells.back().emplace_back(&c);
      c._vpPlaceOrderCstrs.emplace_back(&order);
    }
    else if (_cir._mBaseCellName2Ids.find(cn) != _cir._mBaseCellName2Ids.end()) { // is base cell
      for (const Int cellIdx : _cir._mBaseCellName2Ids.at(cn)) {
        Cell& c = _cir.cell(cellIdx);
        order._vvpCells.back().emplace_back(&c);
        c._vpPlaceOrderCstrs.emplace_back(&order);
      }
    }
    else if (_cir._mSubmoduleName2Ids.find(cn) != _cir._mSubmoduleName2Ids.end()) { // is submodule
      for (const Int cellIdx : _cir._mSubmoduleName2Ids.at(cn)) {
        Cell& c = _cir.cell(cellIdx);
        order._vvpCells.back().emplace_back(&c);
        c._vpPlaceOrderCstrs.emplace_back(&order);
      }
    }
    else if (_cir._mPlaceArrayCstrName2Idx.find(cn) != _cir._mPlaceArrayCstrName2Idx.end()) { // is array cstr
      PlaceArrayCstr& arr = _cir.placeArrayCstr(cn);
      for (Cell* pCell : arr.vpCells()) {
        Cell& c = *pCell;
        order._vvpCells.back().emplace_back(&c);
        c._vpPlaceOrderCstrs.emplace_back(&order);
      }
    }
    else {
      assert(false);
    }
  }

}

void Parser::parseCstrPlaceAlign(const String& name, const YAML::Node& data) 
{
  const Int idx = _cir._vpPlaceAlignCstrs.size();
  _cir._mPlaceAlignCstrName2Idx.emplace(name, idx);
  _cir._vpPlaceAlignCstrs.emplace_back(new PlaceAlignCstr());

  PlaceAlignCstr& align = *_cir._vpPlaceAlignCstrs.back();
 
  align._name = name;
  align._idx = idx;

  // horizontal or vertical align
  const String& axis = data["Axis"].as<String>();
  if (axis == "Vertical") {
    align._isHor = false;
  }
  else if (axis == "Horizontal") {
    align._isHor = true;
  }
  else {
    assert(false);
  }

  // align w.r.t to the low or high edge
  if (data["Edge"]) {
    const String& edge = data["Edge"].as<String>();
    if (edge == "Low") {
      align._isHigh = false;
    }
    else if (edge == "High") {
      align._isHigh = true;
    }
    else {
      assert(false);
    }
  }

  const YAML::Node& contains = data["Contains"];
  for (const YAML::Node& n : contains) {
    const String& cn = n.as<String>();
    if (_cir._mCellName2Idx.find(cn) != _cir._mCellName2Idx.end()) { // is cell
      Cell& c = _cir.cell(cn);
      align._vpCells.emplace_back(&c);
      c._vpPlaceAlignCstrs.emplace_back(&align);
    }
    else if (_cir._mBaseCellName2Ids.find(cn) != _cir._mBaseCellName2Ids.end()) { // is base cell
      for (const Int cellIdx : _cir._mBaseCellName2Ids.at(cn)) {
        Cell& c = _cir.cell(cellIdx);
        align._vpCells.emplace_back(&c);
        c._vpPlaceAlignCstrs.emplace_back(&align);
      }
    }
    else if (_cir._mSubmoduleName2Ids.find(cn) != _cir._mSubmoduleName2Ids.end()) { // is submodule
      for (const Int cellIdx : _cir._mSubmoduleName2Ids.at(cn)) {
        Cell& c = _cir.cell(cellIdx);
        align._vpCells.emplace_back(&c);
        c._vpPlaceAlignCstrs.emplace_back(&align);
      }
    }
    else if (_cir._mPlaceArrayCstrName2Idx.find(cn) != _cir._mPlaceArrayCstrName2Idx.end()) {
      PlaceArrayCstr& arr = _cir.placeArrayCstr(cn);
      align._vpArrays.emplace_back(&arr);
    }
    else if (_cir._mRegionName2Idx.find(cn) != _cir._mRegionName2Idx.end()) { // is region
      Region& reg = _cir.region(cn);
      align._vpRegions.emplace_back(&reg);
    }
    else {
      assert(false);
    }
  }
  assert(align._vpCells.size() == 0 or align._vpRegions.size() == 0); 

}

void Parser::parseCstrPlaceDisjoint(const String& name, const YAML::Node& data) 
{
  const Int idx = _cir._vpPlaceDisjointCstrs.size();
  _cir._mPlaceDisjointCstrName2Idx.emplace(name, idx);
  _cir._vpPlaceDisjointCstrs.emplace_back(new PlaceDisjointCstr());

  PlaceDisjointCstr& dis = *_cir._vpPlaceDisjointCstrs.back();

  dis._name = name;
  dis._idx = idx;

  const String& subtype = data["SubType"].as<String>();
  if (subtype== "Col") {
    dis._isRow = false;
  }
  else if (subtype == "Row") {
    dis._isRow = true;
  }
  else {
    assert(false);
  }

  const YAML::Node& contains = data["Contains"];
  for (const YAML::Node& n : contains) {
    const String& cn = n.as<String>();
    if (_cir._mCellName2Idx.find(cn) != _cir._mCellName2Idx.end()) { // is cell
      Cell& c = _cir.cell(cn);
      dis._vpCells.emplace_back(&c);
      c._vpPlaceDisjointCstrs.emplace_back(&dis);
    }
    else if (_cir._mBaseCellName2Ids.find(cn) != _cir._mBaseCellName2Ids.end()) { // is base cell
      for (const Int cellIdx : _cir._mBaseCellName2Ids.at(cn)) {
        Cell& c = _cir.cell(cellIdx);
        dis._vpCells.emplace_back(&c);
        c._vpPlaceDisjointCstrs.emplace_back(&dis);
      }
    }
    else if (_cir._mSubmoduleName2Ids.find(cn) != _cir._mSubmoduleName2Ids.end()) { // is submodule
      for (const Int cellIdx : _cir._mSubmoduleName2Ids.at(cn)) {
        Cell& c = _cir.cell(cellIdx);
        dis._vpCells.emplace_back(&c);
        c._vpPlaceDisjointCstrs.emplace_back(&dis);
      }
    }
    else {
      assert(false);
    }
  }
} 

void Parser::parseCstrPlaceRow(const String& name, const YAML::Node& data)
{
  const Int idx = _cir._vpPlaceRowCstrs.size();
  _cir._mPlaceRowCstrName2Idx.emplace(name, idx);
  _cir._vpPlaceRowCstrs.emplace_back(new PlaceRowCstr());

  PlaceRowCstr& row = *_cir._vpPlaceRowCstrs.back();

  row._name = name;
  row._idx = idx;

  const String& config = data["Config"].as<String>();
  if (config == "Odd") {
    row._isOdd = true;
  }
  else if (config == "Even") {
    row._isOdd = false;
  }
  else {
    assert(false);
  }

  const YAML::Node& contains = data["Contains"];
  for (const YAML::Node& n : contains) {
    const String& cn = n.as<String>();
    if (_cir._mCellName2Idx.find(cn) != _cir._mCellName2Idx.end()) { // is cell
      Cell& c = _cir.cell(cn);
      row._vpCells.emplace_back(&c);
      c._pPlaceRowCstr = &row;
    }
    else if (_cir._mBaseCellName2Ids.find(cn) != _cir._mBaseCellName2Ids.end()) { // is base cell
      for (const Int cellIdx : _cir._mBaseCellName2Ids.at(cn)) {
        Cell& c = _cir.cell(cellIdx);
        row._vpCells.emplace_back(&c);
        c._pPlaceRowCstr = &row;
      }
    }
    else if (_cir._mSubmoduleName2Ids.find(cn) != _cir._mSubmoduleName2Ids.end()) { // is submodule
      for (const Int cellIdx : _cir._mSubmoduleName2Ids.at(cn)) {
        Cell& c = _cir.cell(cellIdx);
        row._vpCells.emplace_back(&c);
        c._pPlaceRowCstr = &row;
      }
    }
    else {
      assert(false);
    }
  }

}

void Parser::parseCstrRouteMatching(const String& name, const YAML::Node& data, const YAML::Node& grd) 
{
  const Int idx = _cir._vpRouteSymCstrs.size();
  _cir._mRouteSymCstrName2Idx.emplace(name, idx);
  _cir._vpRouteSymCstrs.emplace_back(new RouteSymCstr());

  RouteSymCstr& sym = *_cir._vpRouteSymCstrs.back();

  sym._name = name;
  sym._idx = idx;

  const String& axis = data["Axis"].as<String>();
  if (axis == "Vertical") {
    sym._axis = SymAxisE::ver;
  }
  else if (axis == "Horizontal") {
    sym._axis = SymAxisE::hor;
  }
  else {
    assert(false);
  }

  const YAML::Node& contains = data["Contains"];
  for (const YAML::Node& n : contains) {
    const String& netName = n.as<String>(); 
    Net& net = _cir.net(netName);
    sym._mNetIdx2Idx.emplace(net.idx(), sym._vpNets.size());
    sym._vpNets.emplace_back(&net);
  }

  const YAML::Node& partA = data["PartA"];
  const YAML::Node& partB = data["PartB"];
  assert(partA.size() == partB.size());
  sym._numPartAs = partA.size(); 
  sym._numSelfSyms = sym._vpNets.size() - 2 * partA.size();

  auto addSymPartE = [&] (const String& netName, const SymPartE p) {
    auto it = _cir._mNetName2Idx.find(netName); 
    if (it != _cir._mNetName2Idx.end()) {
      const Net& n = *_cir._vpNets.at(it->second);
      sym._vSymPartEs[sym._mNetIdx2Idx.at(n.idx())] = p;
    }
  };
  auto setSymNets = [&] (const String& na, const String& nb) {
    const Net& neta = _cir.net(na);
    const Net& netb = _cir.net(nb);
    auto naIdxIt = sym._mNetIdx2Idx.find(neta.idx());
    auto nbIdxIt = sym._mNetIdx2Idx.find(netb.idx());
    assert(naIdxIt != sym._mNetIdx2Idx.end());
    assert(nbIdxIt != sym._mNetIdx2Idx.end());
    sym._vSymNetIds[naIdxIt->second] = nbIdxIt->second; 
    sym._vSymNetIds[nbIdxIt->second] = naIdxIt->second; 
  };

  sym._vSymPartEs.resize(sym._vpNets.size(), SymPartE::self);
  sym._vSymNetIds.resize(sym._vpNets.size(), -1);
  for (size_t i = 0; i < partA.size(); ++i) {
    const String& na = partA[i].as<String>();
    const String& nb = partB[i].as<String>();
    addSymPartE(na, SymPartE::a);
    addSymPartE(nb, SymPartE::b);
    setSymNets(na, nb);
  }

  Region& reg = *sym._vpNets.at(0)->_pRegion;
  reg._vpRouteSymCstrs.emplace_back(&sym);
}

void Parser::parseCstrRoutePathMatching(const String& name, const YAML::Node& data, const YAML::Node& grd)
{
  const Int idx = _cir._vpRoutePathMatchCstrs.size();
  _cir._mRoutePathMatchCstrName2Idx.emplace(name, idx);
  _cir._vpRoutePathMatchCstrs.emplace_back(new RoutePathMatchCstr());

  RoutePathMatchCstr& pathMatch = *_cir._vpRoutePathMatchCstrs.back();

  pathMatch._name = name;
  pathMatch._idx = idx;


  const YAML::Node& contains = data["Contains"];
  for (const auto& n : contains) {
    const String& netName = n.as<String>();
    Net* pNet = _cir.pNet(netName);
    pathMatch._mNet2NetIdx.emplace(pNet, pathMatch._vpNets.size());
    pathMatch._vpNets.emplace_back(pNet);
  }

  const YAML::Node& cstrs = data["Cstrs"];
  pathMatch._vPathCstrs.reserve(cstrs.size());
  for (const auto& cstr : cstrs) {
    const Int pathCstrIdx = pathMatch._vPathCstrs.size();
    pathMatch._vPathCstrs.emplace_back();
    auto& pathCstr = pathMatch._vPathCstrs.back();
    pathCstr._idx = pathCstrIdx;

    const YAML::Node& paths = cstr["Paths"];
    for (const auto& pinPair : paths) {
      assert(pinPair.IsSequence() and pinPair.size() == 2);
      const String& srcName = pinPair[0].as<String>();
      const String& tarName = pinPair[1].as<String>();
      Pin* pSrc = _cir.pPin(srcName);
      Pin* pTar = _cir.pPin(tarName);
      if (pSrc->pNet() != pTar->pNet()) {
        spdlog::error("Path matching constraint {} error! Pins {} ({}) and {} ({}) in different nets", name, pSrc->name(), pSrc->pNet()->name(), pTar->name(), pTar->pNet()->name());
        exit(0);
      }
      assert(pSrc->pNet() == pTar->pNet());
      pSrc->pNet()->_vConns.emplace_back(&pathCstr, pathCstr._vConns.size());
      pathCstr._spNets.emplace(pSrc->pNet());
      pathCstr._vConns.emplace_back(pSrc, pTar);
    }
  }

}

void Parser::parseCstrNetPriority(const String& name, const YAML::Node& data)
{
  const Int idx = _cir._vpNetPrioCstrs.size();
  _cir._mNetPrioCstrName2Idx.emplace(name, idx);
  _cir._vpNetPrioCstrs.emplace_back(new NetPrioCstr());

  NetPrioCstr& prio = *_cir._vpNetPrioCstrs.back();

  prio._name = name;
  prio._idx = idx;

  prio._priority = data["Priority"].as<Int>();

  const YAML::Node& contains = data["Contains"];
  for (const YAML::Node& n : contains) {
    const String& netName = n.as<String>();
    Net* pNet = _cir.pNet(netName);
    prio._vpNets.emplace_back(pNet);
    pNet->setType(NetTypeE::critical);
    pNet->setPriority(prio._priority);
  }

  Region& reg = *prio._vpNets.at(0)->_pRegion;
  reg._vpNetPrioCstrs.emplace_back(&prio);
}

Int Parser::toDBUnit(const Real val)
{
  assert(_cir._physRes != 0);
  return std::lround(val / _cir._physRes);
}

Int Parser::toDBUnit2d(const Real val)
{
  assert(_cir._physRes != 0);
  return std::lround(val / _cir._physRes / _cir._physRes);
}

void Parser::parsePlace(const String& fileName)
{
  std::ifstream ifs(fileName, std::ios::in);

  String buf;
  Vector<String> vToks;
  while (std::getline(ifs, buf)) {
    util::str::split(buf, " ", vToks);
    if (vToks[0] == "Bound") {
      assert(vToks.size() == 5);
      _cir.bbox().set(std::stoi(vToks[1]), 
                      std::stoi(vToks[2]),
                      std::stoi(vToks[3]),
                      std::stoi(vToks[4]));
    }
    else if (vToks[0] == "Region") {
      assert(vToks.size() == 6);
      Region& reg = _cir.region(vToks[1]);
      reg.setBboxInner(std::stoi(vToks[2]), std::stoi(vToks[3]), std::stoi(vToks[4]), std::stoi(vToks[5]));
    }
    else if (vToks[0] == "Cell") {
      assert(vToks.size() == 5);
      Cell& c = _cir.cell(vToks[1]);
      assert(c.region().bboxInner().area() > 0);
      const Int x = std::stoi(vToks[2]);
      const Int y = std::stoi(vToks[3]);
      c.setLoc(x, y, 
               (x - c.region().bboxInner().xl()) / c.region().placeGrid().stepX(), 
               (y - c.region().bboxInner().yl()) / c.region().placeGrid().stepY());
      c.setOrient(util::enumUtil::str2Val(Orient2dEStr, vToks[4]));
    }
    else if (vToks[0] == "IOPin") {
      assert(vToks.size() == 6);
      const Int idx = _cir.name2IOPinIdx(vToks[1]);
      const Box<Int> b(std::stoi(vToks[2]), std::stoi(vToks[3]), std::stoi(vToks[4]), std::stoi(vToks[5]));
      _cir.setIOPinBbox(idx, b);
    }
    //else if (vToks[0] == "Edge") {
      //assert(vToks.size() == 7);
      //Region& reg = _cir.region(vToks[1]);
      //const Int idx = reg.numEdgeCells();
      //reg.addEdgeCell();
      //Cell& c = reg.edgeCell(idx);
      //c.setName(vToks[2]);
      //c.setInstName(vToks[2]);
      //c.setIdx(idx);
      //c.setPrim(_cir.prim(vToks[3]));
      //c.setRegion(&reg);
      //c.setLoc(std::stoi(vToks[4]), std::stoi(vToks[5]), c.region().placeGrid().stepX(), c.region().placeGrid().stepY());
      //c.setOrient(util::enumUtil::str2Val(Orient2dEStr, vToks[6]));
    //}
    //else if (vToks[0] == "Fill") {
      //assert(vToks.size() == 7);
      //Region& reg = _cir.region(vToks[1]);
      //const Int idx = reg.numDummyCells();
      //reg.addDummyCell();
      //Cell& c = reg.dummyCell(idx);
      //c.setName(vToks[2]);
      //c.setInstName(vToks[2]);
      //c.setIdx(idx);
      //c.setPrim(_cir.prim(vToks[3]));
      //c.setRegion(&reg);
      //c.setLoc(std::stoi(vToks[4]), std::stoi(vToks[5]), c.region().placeGrid().stepX(), c.region().placeGrid().stepY());
      //c.setOrient(util::enumUtil::str2Val(Orient2dEStr, vToks[6]));
    //}
  }

  //Int i, j;
  //Region* pReg;
  //const Cell* pCell;
  //Cir_ForEachRegion(_cir, pReg, i) {
    //Region& reg = *pReg;
    //Int xl = MAX_INT, xh = MIN_INT;
    //Int yl = MAX_INT, yh = MIN_INT;
    //Reg_ForEachCell(reg, pCell, j) {
      //const Cell& c = *pCell;
      //xl = std::min(xl, c.loc().x());
      //yl = std::min(yl, c.loc().y());
      //xh = std::max(xh, c.loc().x() + c.width());
      //yh = std::max(yh, c.loc().y() + c.height());
    //}
    //Reg_ForEachDummyCell(reg, pCell, j) {
      //const Cell& c = *pCell;
      //xl = std::min(xl, c.loc().x());
      //yl = std::min(yl, c.loc().y());
      //xh = std::max(xh, c.loc().x() + c.width());
      //yh = std::max(yh, c.loc().y() + c.height());
    //}
    //reg.setBboxInner(xl, yl, xh, yh);

    //Reg_ForEachEdgeCell(reg, pCell, j) {
      //const Cell& c = *pCell;
      //xl = std::min(xl, c.loc().x());
      //yl = std::min(yl, c.loc().y());
      //xh = std::max(xh, c.loc().x() + c.width());
      //yh = std::max(yh, c.loc().y() + c.height());
    //}
    //reg.setBboxOuter(xl, yl, xh, yh);
  //}

}

void Parser::parseRouteGds(const String& fileName, const Vector<Int>& vNetIds)
{
  GdsReader gds(_cir);
  gds.parseRouting(fileName, vNetIds);
}

PROJECT_NAMESPACE_END
