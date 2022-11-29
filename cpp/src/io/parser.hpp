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

#include <yaml-cpp/yaml.h>

#include "db/dbCir.hpp"
#include "parHspice.hpp"
#include "parLef.hpp"
#include "parGds.hpp"

PROJECT_NAMESPACE_START


class Parser {
public:
  Parser(CirDB& cir) : _cir(cir) {}

  void parse(const String& laygoConfig,
             const String& techGrid,
             const String& techMap,
             const String& constraint,
             const Vector<String>& vLefs,
             const String& netlist,
             const String& layerRules);

  void parseLef(const Vector<String>& vFileNames);
  void parsePlace(const String& fileName);
  void parseRouteGds(const String& fileName, const Vector<Int>& vNetIds = {});

private:
  CirDB& _cir;

  void parsePhysRes(const YAML::Node& cfg);
  
  void parseLayers(const YAML::Node& cfg, const YAML::Node& drc);
  void parseMetalLayerRules(const YAML::Node& drc, MetalLayer& layer); 
  void parseCutLayerRules(const YAML::Node& drc, CutLayer& layer); 

  Int  parsePlacementGrid(const YAML::Node& data, const YAML::Node& grd);
  
  void parseRoutingGrids(const YAML::Node& data, const YAML::Node& grd, const bool isPower, Vector<Int>& vGridIds);
  void parseRoutingGrid(const YAML::Node& grd, const String& gridName, RouteGrid& rg);
  
  void parseDevPrimMap(const YAML::Node& map);

  // netlist
  void parseNetlist(const String& fileName);
  void parseCells(const HspiceReader& hsr, const HspiceReader::Node& topNode);
  void parseNets(const HspiceReader& hsr, const HspiceReader::Node& topNode);
  void parseNetListPost();

  // constraint: design
  void parseCstrDesign(const String& name, const YAML::Node& data, const YAML::Node& grd);
  
  // constraint: placement - region
  void parseCstrRegion(const String& name, const YAML::Node& data, const YAML::Node& grd);
  void parseEdgePrims(const YAML::Node& data, Region& reg);
  void parseDummyPrims(const YAML::Node& data, Region& reg);

  // constraint: placement - symmetry
  void parseCstrPlaceSymmetry(const String& name, const YAML::Node& data);
  // constraint: placement - array
  void parseCstrPlaceArray(const String& name, const YAML::Node& data);
  // constraint: placement - cluster
  void parseCstrPlaceCluster(const String& name, const YAML::Node& data);
  // constraint: placement - preplace
  void parseCstrPlacePreplace(const String& name, const YAML::Node& data);
  // constraint: placement - (io)pin assignment
  void parseCstrPinAssignment(const String& name, const YAML::Node& data, const YAML::Node& grd);
  // constraint: placement - extension
  void parseCstrPlaceExtension(const String& name, const YAML::Node& data);
  // constraint: placement - edge dist
  void parseCstrPlaceEdgeDist(const String& name, const YAML::Node& data);
  // constraint: placement - order 
  void parseCstrPlaceOrder(const String& name, const YAML::Node& data);
  // constraint: placement - align 
  void parseCstrPlaceAlign(const String& name, const YAML::Node& data);
  // constraint: placement - disjoint 
  void parseCstrPlaceDisjoint(const String& name, const YAML::Node& data);
  // constraint: placement - row 
  void parseCstrPlaceRow(const String& name, const YAML::Node& data);
  
  // constraint: routing - net assignment
  void parseCstrNetAssignment(const String& name, const YAML::Node& data, const YAML::Node& grd);
  // constraint: routing - power
  void parseCstrPower(const String& name, const YAML::Node& data);
  // constraint: routing - matching
  void parseCstrRouteMatching(const String& name, const YAML::Node& data, const YAML::Node& grd);
  void parseCstrRoutePathMatching(const String& name, const YAML::Node& data, const YAML::Node& grd);
  // constraint: routing - priority
  void parseCstrNetPriority(const String& name, const YAML::Node& data);

  // helper
  Int toDBUnit(const Real val);
  Int toDBUnit2d(const Real val);

  //const FlatHashSet<String> _sVddNames{"vdd", "vddl"};
  //const FlatHashSet<String> _sVssNames{"gnd"};

};

PROJECT_NAMESPACE_END
