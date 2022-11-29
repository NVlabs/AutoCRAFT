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

#include <unistd.h>
#include <lefiDebug.hpp>
#include <lefiEncryptInt.hpp>
#include <lefiUtil.hpp>
#include <lefrReader.hpp>

#include "global/global.hpp"
#include "db/dbCir.hpp"

PROJECT_NAMESPACE_START

class LefReader {
 public:
  LefReader(CirDB& cir) : _cir(cir), _dbUnit(2000) {}
  ~LefReader() {}

  void parse(const Vector<String>& vFileNames);

  // callback impls
  void parseManufacturing(const Real num);
  void parseUnits(const lefiUnits* p);

  void parsePropDefs(const lefiProp* p);

  void parseLayer(const lefiLayer* p);
  void parseRoutingLayer(const lefiLayer* p);
  void parseCutLayer(const lefiLayer* p);

  void parseMacroBegin(const String& macroName);
  void parseMacroClassType(const String& macroClassType);
  void parseMacroOrigin(const lefiNum& macroNum);
  void parseMacroSize(const lefiNum& macroNum);
  void parseMacroForeign(const lefiMacroForeign* p);
  void parseMacroSite(const lefiMacroSite* p);
  void parseMacro(const lefiMacro* p);
  void parseMacroEnd(const String& macroName);
  void parsePin(const lefiPin* p);
  void parseObstruction(const lefiObstruction* p);
  void parsePinGeometry(Pin& pin, const Int originX, const Int originY, const lefiGeometries* pGeo);
  void parseObsGeometry(const lefiGeometries* pGeo);
  void parseVia(const lefiVia* p);


 private:
  CirDB& _cir;

  FlatHashMap<String, Int> _mLibraryPropName2Idx;
  Vector<char>             _vLibraryPropDataTypes;
  Vector<Pair<Int, Int>>   _vLibraryPropIntRanges;
  Vector<Pair<Real, Real>> _vLibraryPropRealRanges;
  Vector<Int>              _vLibraryPropNumberValues;
  Vector<String>           _vLibraryPropStringValues;

  FlatHashMap<String, Int> _mLayerPropName2Idx;
  Vector<char>             _vLayerPropDataTypes;
  Vector<Pair<Int, Int>>   _vLayerPropIntRanges;
  Vector<Pair<Real, Real>> _vLayerPropRealRanges;
  Vector<Int>              _vLayerPropNumberValues;

  FlatHashMap<String, Int> _mMacroPropName2Idx;
  Vector<char>             _vMacroPropDataTypes;
  Vector<Pair<Int, Int>>   _vMacroPropIntRanges;
  Vector<Pair<Real, Real>> _vMacroPropRealRanges;
  Vector<Int>              _vMacroPropNumberValues;
  
  FlatHashMap<String, Int> _mNonDefaultPropName2Idx;
  Vector<char>             _vNonDefaultPropDataTypes;
  Vector<Pair<Int, Int>>   _vNonDefaultPropIntRanges;
  Vector<Pair<Real, Real>> _vNonDefaultPropRealRanges;
  Vector<Int>              _vNonDefaultPropNumberValues;

  FlatHashMap<String, Int> _mPinPropName2Idx;
  Vector<char>             _vPinPropDataTypes;
  Vector<Pair<Int, Int>>   _vPinPropIntRanges;
  Vector<Pair<Real, Real>> _vPinPropRealRanges;
  Vector<Int>              _vPinPropNumberValues;

  FlatHashMap<String, Int> _mViaPropName2Idx;
  Vector<char>             _vViaPropDataTypes;
  Vector<Pair<Int, Int>>   _vViaPropIntRanges;
  Vector<Pair<Real, Real>> _vViaPropRealRanges;
  Vector<Int>              _vViaPropNumberValues;

  FlatHashMap<String, Int> _mViaRulePropName2Idx;
  Vector<char>             _vViaRulePropDataTypes;
  Vector<Pair<Int, Int>>   _vViaRulePropIntRanges;
  Vector<Pair<Real, Real>> _vViaRulePropRealRanges;
  Vector<Int>              _vViaRulePropNumberValues;

  Int toDBUnit(const Real val) const;
  Int toDBUnit2d(const Real val) const;
  Int _dbUnit;
  
  // LEF58
  enum class Lef58SpacingType1 {
    undef,
    EOLPERPENDICULAR,
    AREA,
    LAYER,
    NOTCHLENGTH,
    NOTCHSPAN,
    ENDOFLINE,
    CONVEXCORNERS
  };
  enum class Lef58SpacingType2 {
    undef,
    TOCONCAVECORNER,
    TONOTCHLENGTH
  };
  void parseLef58Area(MetalLayer& layer, const String& s);
  void parseLef58Spacing(MetalLayer& layer, const String& s);
  void parseLef58SpacingEol(MetalLayer& layer, const String& s);
  //void parseLef58SpacingTable(MetalLayer& layer, const String& s);
  //void parseLef58SpacingTablePrl(MetalLayer& layer, const String& s);
  void parseLef58EolKeepOut(MetalLayer& layer, const String& s);
  void parseLef58MinStep(MetalLayer& layer, const String& s);
  void parseLef58WidthTable(MetalLayer& layer, const String& s);
};

PROJECT_NAMESPACE_END
