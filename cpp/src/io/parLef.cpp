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

#include "parLef.hpp"

PROJECT_NAMESPACE_START

namespace lef {
  LefReader*  pLef            = nullptr;
  Real        lefVersion      = 0;
  String      lefVersionStr   = "";
  String      lefBusBitChars  = "";
  String      lefDividerChar  = "";
  Int         lefDbUnit       = 0;

  void errorCB(const char* msg)
  {
    printf("%s : %s\n", (char*)lefrGetUserData(), msg);
  }

  void warningCB(const char* msg)
  {
    printf("%s : %s\n", (char*)lefrGetUserData(), msg);
  }

  void* mallocCB(int size)
  {
    return malloc(size);
  }

  void* reallocCB(void* name, int size)
  {
    return realloc(name, size);
  }

  void freeCB(void* name)
  {
    free(name);
  }

  void lineNumberCB(int lineNo)
  {
    fprintf(stderr, "Parsed %d number of lines!!\n", lineNo);
  }

  void printWarning(const char* str)
  {
    fprintf(stderr, "%s\n", str);
  }

  void dataError() {
    fprintf(stderr, "ERROR: returned user data is not correct!\n");
  }

  void checkType(lefrCallbackType_e c)
  {
    if (c >= 0 && c <= lefrLibraryEndCbkType) {
      // OK
    }
    else {
      fprintf(stderr, "ERROR: callback type is out of bounds!\n");
    }
  }

  char* orientStr(int orient)
  {
    switch (orient) {
      case 0: return ((char*)"N");
      case 1: return ((char*)"W");
      case 2: return ((char*)"S");
      case 3: return ((char*)"E");
      case 4: return ((char*)"FN");
      case 5: return ((char*)"FW");
      case 6: return ((char*)"FS");
      case 7: return ((char*)"FE");
    };
    return ((char*)"BOGUS");
  }

  int versionCB(lefrCallbackType_e c, double num, lefiUserData)
  {
    checkType(c);
    lefVersion = num;
    return 0;
  }

  int versionStrCB(lefrCallbackType_e c, const char* versionName, lefiUserData)
  {
    checkType(c);
    lefVersionStr = versionName;
    return 0;
  }

  int busBitCharsCB(lefrCallbackType_e c, const char* busBit, lefiUserData)
  {
    checkType(c);
    lefBusBitChars = busBit;
    return 0;
  }

  int dividerCB(lefrCallbackType_e c, const char* name, lefiUserData)
  {
    checkType(c);
    lefDividerChar = name;
    return 0;
  }

  int manufacturingCB(lefrCallbackType_e c, double num, lefiUserData)
  {
    checkType(c);
    pLef->parseManufacturing(num);
    return 0;
  }

  int unitCB(lefrCallbackType_e c, lefiUnits* units, lefiUserData)
  {
    checkType(c);
    pLef->parseUnits(units);
    return 0;
  }

  int layerCB(lefrCallbackType_e c, lefiLayer* layer, lefiUserData)
  {
    checkType(c);
    pLef->parseLayer(layer);
    return 0;
  }

  int macroBeginCB(lefrCallbackType_e c, const char* macroName, lefiUserData)
  {
    checkType(c);
    pLef->parseMacroBegin(macroName);
    return 0;
  }

  int macroClassTypeCB(lefrCallbackType_e c, const char* macroClassType, lefiUserData)
  {
    checkType(c);
    pLef->parseMacroClassType(macroClassType);
    return 0;
  }

  int macroOriginCB(lefrCallbackType_e c, lefiNum macroNum, lefiUserData)
  {
    checkType(c);
    pLef->parseMacroOrigin(macroNum);
    return 0;
  }

  int macroSizeCB(lefrCallbackType_e c, lefiNum macroNum, lefiUserData)
  {
    checkType(c);
    pLef->parseMacroSize(macroNum);
    return 0;
  }
  
  int macroForeignCB(lefrCallbackType_e c, const lefiMacroForeign* foreign, lefiUserData)
  {
    checkType(c);
    pLef->parseMacroForeign(foreign);
    return 0;
  }
  
  int macroSiteCB(lefrCallbackType_e c, const lefiMacroSite* foreign, lefiUserData)
  {
    checkType(c);
    pLef->parseMacroSite(foreign);
    return 0;
  }

  int macroCB(lefrCallbackType_e c, lefiMacro* macro, lefiUserData)
  {
    checkType(c);
    pLef->parseMacro(macro);
    return 0;
  }

  int macroEndCB(lefrCallbackType_e c, const char* macroName, lefiUserData)
  {
    checkType(c);
    pLef->parseMacroEnd(macroName);
    return 0;
  }


  int obstructionCB(lefrCallbackType_e c, lefiObstruction* obs, lefiUserData)
  {
    checkType(c);
    pLef->parseObstruction(obs);
    return 0;
  }

  int pinCB(lefrCallbackType_e c, lefiPin* pin, lefiUserData)
  {
    checkType(c);
    pLef->parsePin(pin);
    return 0;
  }

  int viaCB(lefrCallbackType_e c, lefiVia* via, lefiUserData)
  {
    checkType(c);
    pLef->parseVia(via);
    return 0;
  }

  int propDefBeginCB(lefrCallbackType_e c, void*, lefiUserData)
  {
    checkType(c);
    return 0;
  }

  int propDefCB(lefrCallbackType_e c, lefiProp* prop, lefiUserData)
  {
    checkType(c);
    pLef->parsePropDefs(prop);
    return 0;
  }

  int propDefEndCB(lefrCallbackType_e c, void*, lefiUserData)
  {
    checkType(c);
    return 0;
  }


  int doneCB(lefrCallbackType_e c, void*, lefiUserData)
  {
    checkType(c);
    return 0;
  }

}

void LefReader::parseManufacturing(const Real num)
{
  if (_cir._physRes != 0) {
    assert(std::fabs(_cir._physRes - num) < EPSILON);
  }
  else {
    _cir._physRes = num;
  }
}

void LefReader::parseUnits(const lefiUnits* p)
{
  if (p->lefiUnits::hasDatabase()) {
    _dbUnit = p->lefiUnits::databaseNumber();
  }
}

void LefReader::parsePropDefs(const lefiProp* p)
{
  if (strcmp(p->lefiProp::propType(), "library") == 0) {
    _mLibraryPropName2Idx.emplace(p->lefiProp::propName(), _vLibraryPropDataTypes.size());
    _vLibraryPropDataTypes.emplace_back(p->lefiProp::dataType());
    _vLibraryPropIntRanges.emplace_back(p->lefiProp::hasRange() ? std::make_pair(static_cast<Int>(p->lefiProp::left()), static_cast<Int>(p->lefiProp::right())) : std::make_pair(MIN_INT, MAX_INT));
    _vLibraryPropRealRanges.emplace_back(p->lefiProp::hasRange() ? std::make_pair(p->lefiProp::left(), p->lefiProp::right()) : std::make_pair(MIN_REAL, MAX_REAL));
    _vLibraryPropNumberValues.emplace_back(p->lefiProp::hasNumber() ? p->lefiProp::number() : 0);
    _vLibraryPropStringValues.emplace_back(p->lefiProp::hasString() ? p->lefiProp::string() : "");
  }
  else if (strcmp(p->lefiProp::propType(), "layer") == 0) {
    _mLayerPropName2Idx.emplace(p->lefiProp::propName(), _vLayerPropDataTypes.size());
    _vLayerPropDataTypes.emplace_back(p->lefiProp::dataType());
    _vLayerPropIntRanges.emplace_back(p->lefiProp::hasRange() ? std::make_pair(static_cast<Int>(p->lefiProp::left()), static_cast<Int>(p->lefiProp::right())) : std::make_pair(MIN_INT, MAX_INT));
    _vLayerPropRealRanges.emplace_back(p->lefiProp::hasRange() ? std::make_pair(p->lefiProp::left(), p->lefiProp::right()) : std::make_pair(MIN_REAL, MAX_REAL));
    _vLayerPropNumberValues.emplace_back(p->lefiProp::hasNumber() ? p->lefiProp::number() : 0);
  }
  else if (strcmp(p->lefiProp::propType(),"macro") == 0) {
    _mMacroPropName2Idx.emplace(p->lefiProp::propName(), _vMacroPropDataTypes.size());
    _vMacroPropDataTypes.emplace_back(p->lefiProp::dataType());
    _vMacroPropIntRanges.emplace_back(p->lefiProp::hasRange() ? std::make_pair(static_cast<Int>(p->lefiProp::left()), static_cast<Int>(p->lefiProp::right())) : std::make_pair(MIN_INT, MAX_INT));
    _vMacroPropRealRanges.emplace_back(p->lefiProp::hasRange() ? std::make_pair(p->lefiProp::left(), p->lefiProp::right()) : std::make_pair(MIN_REAL, MAX_REAL));
    _vMacroPropNumberValues.emplace_back(p->lefiProp::hasNumber() ? p->lefiProp::number() : 0);
  }
  else if (strcmp(p->lefiProp::propType(), "nondefaultrule") == 0) {
    _mNonDefaultPropName2Idx.emplace(p->lefiProp::propName(), _vNonDefaultPropDataTypes.size());
    _vNonDefaultPropDataTypes.emplace_back(p->lefiProp::dataType());
    _vNonDefaultPropIntRanges.emplace_back(p->lefiProp::hasRange() ? std::make_pair(static_cast<Int>(p->lefiProp::left()), static_cast<Int>(p->lefiProp::right())) : std::make_pair(MIN_INT, MAX_INT));
    _vNonDefaultPropRealRanges.emplace_back(p->lefiProp::hasRange() ? std::make_pair(p->lefiProp::left(), p->lefiProp::right()) : std::make_pair(MIN_REAL, MAX_REAL));
    _vNonDefaultPropNumberValues.emplace_back(p->lefiProp::hasNumber() ? p->lefiProp::number() : 0);
  }
  else if (strcmp(p->lefiProp::propType(),"pin") == 0) {
    _mPinPropName2Idx.emplace(p->lefiProp::propName(), _vPinPropDataTypes.size());
    _vPinPropDataTypes.emplace_back(p->lefiProp::dataType());
    _vPinPropIntRanges.emplace_back(p->lefiProp::hasRange() ? std::make_pair(static_cast<Int>(p->lefiProp::left()), static_cast<Int>(p->lefiProp::right())) : std::make_pair(MIN_INT, MAX_INT));
    _vPinPropRealRanges.emplace_back(p->lefiProp::hasRange() ? std::make_pair(p->lefiProp::left(), p->lefiProp::right()) : std::make_pair(MIN_REAL, MAX_REAL));
    _vPinPropNumberValues.emplace_back(p->lefiProp::hasNumber() ? p->lefiProp::number() : 0);
  }
  else if (strcmp(p->lefiProp::propType(), "via") == 0) {
    _mViaPropName2Idx.emplace(p->lefiProp::propName(), _vViaPropDataTypes.size());
    _vViaPropDataTypes.emplace_back(p->lefiProp::dataType());
    _vViaPropIntRanges.emplace_back(p->lefiProp::hasRange() ? std::make_pair(static_cast<Int>(p->lefiProp::left()), static_cast<Int>(p->lefiProp::right())) : std::make_pair(MIN_INT, MAX_INT));
    _vViaPropRealRanges.emplace_back(p->lefiProp::hasRange() ? std::make_pair(p->lefiProp::left(), p->lefiProp::right()) : std::make_pair(MIN_REAL, MAX_REAL));
    _vViaPropNumberValues.emplace_back(p->lefiProp::hasNumber() ? p->lefiProp::number() : 0);
  }
  else if (strcmp(p->lefiProp::propType(), "viarule") == 0) {
    _mViaRulePropName2Idx.emplace(p->lefiProp::propName(), _vViaRulePropDataTypes.size());
    _vViaRulePropDataTypes.emplace_back(p->lefiProp::dataType());
    _vViaRulePropIntRanges.emplace_back(p->lefiProp::hasRange() ? std::make_pair(static_cast<Int>(p->lefiProp::left()), static_cast<Int>(p->lefiProp::right())) : std::make_pair(MIN_INT, MAX_INT));
    _vViaRulePropRealRanges.emplace_back(p->lefiProp::hasRange() ? std::make_pair(p->lefiProp::left(), p->lefiProp::right()) : std::make_pair(MIN_REAL, MAX_REAL));
    _vViaRulePropNumberValues.emplace_back(p->lefiProp::hasNumber() ? p->lefiProp::number() : 0);
  }
  else {
    spdlog::warn("[LefReader] PROPTYPE \"{}\" not supported", p->lefiProp::propType());
  }
}

void LefReader::parseLayer(const lefiLayer* p)
{
  if (p->lefiLayer::hasType()) {
    if (strcmp(p->lefiLayer::type(), "ROUTING") == 0) {
      parseRoutingLayer(p);
    }
    else if (strcmp(p->lefiLayer::type(), "CUT") == 0) {
      parseCutLayer(p);
    }
    else {
      assert(false);
    }
  }
}

void LefReader::parseRoutingLayer(const lefiLayer* p)
{
  const Int idx = _cir._vpLayers.size();
  const Int selfIdx = _cir._vpMetalLayers.size();
  const String& name = p->lefiLayer::name();

  _cir._vpMetalLayers.emplace_back(new MetalLayer());
  _cir._mMetalLayerName2Idx.emplace(name, selfIdx);
  _cir._vpLayers.emplace_back(_cir._vpMetalLayers.back());
  _cir._mLayerName2Idx.emplace(name, idx);

  MetalLayer& layer = *_cir._vpMetalLayers.back();

  layer._name = name;
  layer._idx = idx;
  layer._selfIdx = selfIdx;
  
  if (p->lefiLayer::hasDirection()) {
    if (strcmp(p->lefiLayer::direction(), "HORIZONTAL") == 0) {
      layer._direction = RoutePrefE::hor;
    }
    else if (strcmp(p->lefiLayer::direction(), "VERTICAL") == 0) { 
      layer._direction = RoutePrefE::ver;
    }
    else {
      assert(false);
    }
  }
  if (p->lefiLayer::hasPitch()) {
    layer._pitchX = layer._pitchY = toDBUnit(p->lefiLayer::pitch());
  }
  if (p->lefiLayer::hasXYPitch()) {
    layer._pitchX = toDBUnit(p->lefiLayer::pitchX());
    layer._pitchY = toDBUnit(p->lefiLayer::pitchY());
  }
  if (p->lefiLayer::hasOffset()) {
    layer._offsetX = layer._offsetY = toDBUnit(p->lefiLayer::offset());
  }
  if (p->lefiLayer::hasXYOffset()) {
    layer._offsetX = toDBUnit(p->lefiLayer::offsetX());
    layer._offsetY = toDBUnit(p->lefiLayer::offsetY());
  }
  if (p->lefiLayer::hasArea()) {
    layer._area = toDBUnit2d(p->lefiLayer::area());
  }
  if (p->lefiLayer::hasWidth()) {
    layer._width = layer._minWidth = toDBUnit(p->lefiLayer::width());
  }
  if (p->lefiLayer::hasMinwidth()) {
    layer._minWidth = toDBUnit(p->lefiLayer::minwidth());
  }
  if (p->lefiLayer::hasMaxwidth()) {
    layer._maxWidth = toDBUnit(p->lefiLayer::maxwidth());
  }
  if (p->lefiLayer::hasSpacingNumber()) {
    for (Int i = 0; i < p->lefiLayer::numSpacing(); ++i) {
      const Int spacing = toDBUnit(p->lefiLayer::spacing(i));
      if (p->lefiLayer::hasSpacingRange(i)) {
        spdlog::warn("[LefReader] LAYER {} SPACING RANGE not supported", layer._name);
      }
      else if (p->lefiLayer::hasSpacingLengthThreshold(i)) {
        spdlog::warn("[LefReader] LAYER {} SPACING LENGTHTHRESHOLD not supported", layer._name);
      }
      else if (p->lefiLayer::hasSpacingNotchLength(i)) {
        layer._notchLength = p->lefiLayer::spacingNotchLength(i);
        layer._notchSpacing = spacing;
      }
      else if (p->lefiLayer::hasSpacingEndOfLine(i)) {
        layer._vEolSpaces.emplace_back(spacing);
        layer._vEolWidths.emplace_back(toDBUnit(p->lefiLayer::spacingEolWidth(i)));
        layer._vEolWithins.emplace_back(toDBUnit(p->lefiLayer::spacingEolWithin(i)));
        if (p->lefiLayer::hasSpacingParellelEdge(i)) {
          layer._vEolParSpaces.emplace_back(toDBUnit(p->lefiLayer::spacingParSpace(i)));
          layer._vEolParWithins.emplace_back(toDBUnit(p->lefiLayer::spacingParWithin(i)));
          layer._vEolHasTwoEdges.emplace_back(p->lefiLayer::hasSpacingTwoEdges(i));
        }
        else {
          layer._vEolParSpaces.emplace_back(0);
          layer._vEolParWithins.emplace_back(0);
          layer._vEolHasTwoEdges.emplace_back(false);
        }
      }
      else if (p->lefiLayer::hasSpacingSamenet(i)) {
        if (layer._sameNetSpacing) {
          spdlog::warn("[LefReader] LAYER {} SPACING SAMENET overrided", layer._name);
        }
        layer._sameNetSpacing = spacing;
      }
      else if (p->lefiLayer::hasSpacingSamenetPGonly(i)) {
        if (layer._sameNetSpacingPG) {
          spdlog::warn("[LefReader] LAYER {} SPACING SAMENET (PG) overrided", layer._name);
        }
        layer._sameNetSpacingPG = spacing;
      }
      else { // default min spacing
        if (layer._spacing) {
          spdlog::warn("[LefReader] LAYER {} SPACING overrided", layer._name);
        }
        layer._spacing = spacing;
      }
      
    }
    // sort eol spacing
    if (layer._vEolSpaces.size() > 0) {
      for (size_t i = 0; i < layer._vEolSpaces.size() - 1; ++i) {
        for (size_t j = i + 1; j < layer._vEolSpaces.size(); ++j) {
          if (layer._vEolWidths.at(i) > layer._vEolWidths.at(j)) {
            std::swap(layer._vEolSpaces.at(i), layer._vEolSpaces.at(j));
            std::swap(layer._vEolWidths.at(i), layer._vEolWidths.at(j));
            std::swap(layer._vEolWithins.at(i), layer._vEolWithins.at(j));
            std::swap(layer._vEolParSpaces.at(i), layer._vEolParSpaces.at(j));
            std::swap(layer._vEolParWithins.at(i), layer._vEolParWithins.at(j));
            std::swap(layer._vEolHasTwoEdges.at(i), layer._vEolHasTwoEdges.at(j));
          }
        }
      }
    }
  }
  for (Int i = 0; i < const_cast<lefiLayer*>(p)->lefiLayer::numSpacingTable(); ++i) {
    const lefiSpacingTable* spTable = const_cast<lefiLayer*>(p)->lefiLayer::spacingTable(i);
    if (spTable->lefiSpacingTable::isInfluence()) {
      spdlog::warn("[LefReader] LAYER {} SPACINGTABLE INFLUENCE not suported", layer._name);
    }
    else if (spTable->lefiSpacingTable::isParallel()) {
      if (!layer._vSpacingTablePrl.empty()) {
        spdlog::warn("[LefReader] LAYER {} SPACINGTABLE PARALLEL overrided", layer._name);
      }
      const lefiParallel* parallel = spTable->lefiSpacingTable::parallel();
      layer._spacingTable.resize(parallel->lefiParallel::numWidth(), parallel->lefiParallel::numLength());
      for (Int j = 0; j < parallel->lefiParallel::numLength(); ++j) {
        layer._vSpacingTablePrl.emplace_back(toDBUnit(parallel->lefiParallel::length(j)));
      }
      for (Int j = 0; j < parallel->lefiParallel::numWidth(); ++j) {
        layer._vSpacingTableWidth.emplace_back(toDBUnit(parallel->lefiParallel::width(j)));
        for (Int k = 0; k < parallel->lefiParallel::numLength(); ++k) {
          layer._spacingTable.set(j, k, parallel->lefiParallel::widthSpacing(j, k));
        }
      }
    }
    else { // two widths
      spdlog::warn("[LefReader] LAYER {} SPACINGTABLE TWOWIDTHS not supported", layer._name);
    }

  }
  if (p->lefiLayer::hasDiagSpacing()) {
    layer._diagSpacing = toDBUnit(p->lefiLayer::diagSpacing());
  }
  if (p->lefiLayer::hasDiagMinEdgeLength()) {
    layer._diagMinEdgeLength = toDBUnit(p->lefiLayer::diagMinEdgeLength());
  }
  for (Int i = 0; i < p->lefiLayer::numProps(); ++i) {
    const String& propName = p->lefiLayer::propName(i);
    const Int propDefIdx = _mLayerPropName2Idx.at(propName);
    const char propType = p->lefiLayer::propType(i);
    if (propType != _vLayerPropDataTypes.at(propDefIdx)) {
      spdlog::error("[LefReader] LAYER {} PROPERTY {} type mismatch", layer._name, propName);
      exit(0);
    }
    if (p->lefiLayer::propIsNumber(i)) {
      spdlog::warn("[LefReader] LAYER {} PROPERTY {} not supported", layer._name, propName);
      switch (propType) {
        case 'R': {
          const auto& range = _vLayerPropRealRanges.at(propDefIdx);
          if (p->lefiLayer::propNumber(i) < range.first or
              p->lefiLayer::propNumber(i) > range.second) {
            spdlog::error("[LefReader] LAYER {} PROPERTY {} number out of range", layer._name, propName);
            exit(0);
          }
          break;
        }
        case 'I': {
          const auto& range = _vLayerPropIntRanges.at(propDefIdx);
          if (p->lefiLayer::propNumber(i) < range.first or
              p->lefiLayer::propNumber(i) > range.second) {
            spdlog::error("[LefReader] LAYER {} PROPERTY {} number out of range", layer._name, propName);
            exit(0);
          }
          break;
        }
        default:
          assert(false);
      }
    }
    else if (p->lefiLayer::propIsString(i)) {
      if (propName == "LEF58_SPACING") {
        spdlog::warn("[LefReader] LAYER {} PROPERTY {} not supported", layer._name, propName);
        parseLef58Spacing(layer, p->lefiLayer::propValue(i));
      }
      else if (propName == "LEF58_MINSTEP") {
        spdlog::warn("[LefReader] LAYER {} PROPERTY {} not supported", layer._name, propName);
        parseLef58MinStep(layer, p->lefiLayer::propValue(i));
      }
      else if (propName == "LEF58_WIDTHTABLE") {
        spdlog::warn("[LefReader] LAYER {} PROPERTY {} not supported", layer._name, propName);
        parseLef58WidthTable(layer, p->lefiLayer::propValue(i));
      }
      else if (propName == "LEF58_EOLKEEPOUT") {
        spdlog::warn("[LefReader] LAYER {} PROPERTY {} not supported", layer._name, propName);
        parseLef58EolKeepOut(layer, p->lefiLayer::propValue(i));
      }
      else if (propName == "LEF58_AREA") {
        spdlog::warn("[LefReader] LAYER {} PROPERTY {} not supported", layer._name, propName);
        parseLef58Area(layer, p->lefiLayer::propValue(i));
      }
      else {
        spdlog::warn("[LefReader] LAYER {} PROPERTY {} not supported", layer._name, propName);
      }
    }

  }
  
}

void LefReader::parseCutLayer(const lefiLayer* p)
{
  const Int idx = _cir._vpLayers.size();
  const Int selfIdx = _cir._vpCutLayers.size();
  const String& name = p->lefiLayer::name();

  _cir._vpCutLayers.emplace_back(new CutLayer());
  _cir._mCutLayerName2Idx.emplace(name, selfIdx);
  _cir._vpLayers.emplace_back(_cir._vpCutLayers.back());
  _cir._mLayerName2Idx.emplace(name, idx);

  CutLayer& layer = *_cir._vpCutLayers.back();
  
  layer._name = name;
  layer._idx = idx;
  layer._selfIdx = selfIdx;
}


void LefReader::parseMacroBegin(const String& macroName)
{
  const Int idx = _cir._vpPrims.size();
  _cir._mPrimName2Idx.emplace(macroName, idx); 
  _cir._vpPrims.emplace_back(new Primitive());
  Primitive& prim = *_cir._vpPrims.back();
  prim._name = macroName;
  prim._idx = idx;
}

void LefReader::parseMacroClassType(const String& macroClassType)
{
  Primitive& prim = *_cir._vpPrims.back();
  prim._classType = macroClassType;
}

void LefReader::parseMacroOrigin(const lefiNum& macroNum)
{
  Primitive& prim = *_cir._vpPrims.back();
  prim._originX = toDBUnit(macroNum.x);
  prim._originY = toDBUnit(macroNum.y);
}

void LefReader::parseMacroSize(const lefiNum& macroNum)
{
  Primitive& prim = *_cir._vpPrims.back();
  prim._sizeX = toDBUnit(macroNum.x);
  prim._sizeY = toDBUnit(macroNum.y);
}

void LefReader::parseMacroForeign(const lefiMacroForeign* p)
{
  Primitive& prim = *_cir._vpPrims.back();
  prim._foreignCellName = p->lefiMacroForeign::cellName();
  if (p->lefiMacroForeign::cellHasPts()) {
    prim._foreignX = toDBUnit(p->lefiMacroForeign::px());
    prim._foreignY = toDBUnit(p->lefiMacroForeign::py());
  }
  else {
    prim._foreignX = 0;
    prim._foreignY = 0;
  }
}

void LefReader::parseMacroSite(const lefiMacroSite* p)
{

}

void LefReader::parseMacro(const lefiMacro* p)
{
  Primitive& prim = *_cir._vpPrims.back();
  if (p->lefiMacro::hasXSymmetry()) {
    prim._isXSym = true;
  }
  if (p->lefiMacro::hasYSymmetry()) {
    prim._isYSym = true;
  }
  if (p->lefiMacro::has90Symmetry()) {
    prim._is90Sym = true;
  }
}

void LefReader::parseMacroEnd(const String& macroName)
{

}

void LefReader::parsePin(const lefiPin* p)
{
  Primitive& prim = *_cir._vpPrims.back();
  prim._mPinName2Idx.emplace(p->lefiPin::name(), prim._vPins.size());
  prim._vPins.emplace_back();

  Pin& pin = prim._vPins.back();
  pin._name = p->lefiPin::name();
  
  if (p->lefiPin::hasDirection()) {
    if (strcmp(p->lefiPin::direction(), "INPUT") == 0) {
      pin._dir = PinDirE::input;
    }
    else if (strcmp(p->lefiPin::direction(), "OUTPUT") == 0) {
      pin._dir = PinDirE::output;
    }
    else if (strcmp(p->lefiPin::direction(), "INOUT") == 0) {
      pin._dir = PinDirE::inout;
    }
    else if (strcmp(p->lefiPin::direction(), "FEEDTHRU") == 0) {
      pin._dir = PinDirE::feedthru;
    }
    else {
      assert(false);
    }
  }
  if (p->lefiPin::hasShape()) {
    if (strcmp(p->lefiPin::shape(), "ABUTMENT") == 0) {
      pin._shape = PinShapeE::abutment;
    }
    else if (strcmp(p->lefiPin::shape(), "RING") == 0) {
      pin._shape = PinShapeE::ring;
    }
    else if (strcmp(p->lefiPin::shape(), "FEEDTHRU") == 0) {
      pin._shape = PinShapeE::feedthru;
    }
    else {
      assert(false);
    }
  }
  //if (pLefPin->lefiPin::hasUse()) {
    //if (strcmp(pLefPin->lefiPin::use(), "SIGNAL") == 0) {
      //pin._use = PinUse::signal;
    //}
    //else if (strcmp(pLefPin->lefiPin::use(), "ANALOG") == 0) {
      //pin._use = PinUse::analog;
    //}
    //else if (strcmp(pLefPin->lefiPin::use(), "POWER") == 0) {
      //pin._use = PinUse::power;
    //}
    //else if (strcmp(pLefPin->lefiPin::use(), "GROUND") == 0) {
      //pin._use = PinUse::ground;
    //}
    //else if (strcmp(pLefPin->lefiPin::use(), "CLOCK") == 0) {
      //pin._use = PinUse::clock;
    //}
    //else {
      //assert(false);
    //}
  //}

  for (Int i = 0; i < p->lefiPin::numPorts(); ++i) {
    lefiGeometries* pGeo = p->lefiPin::port(i);
    parsePinGeometry(pin, prim._originX, prim._originY,  pGeo);
  }

}

void LefReader::parsePinGeometry(Pin& pin, const Int originX, const Int originY, const lefiGeometries* pGeo)
{
  
  Int layerIdx = -1;
  String layerName;
  lefiGeomRect* lefRect = nullptr;
  Int xl = 0, yl = 0, xh = 0, yh = 0;
  
  pin._vvBoxIds.resize(_cir.numLayers());
  for (Int i = 0; i < pGeo->lefiGeometries::numItems(); ++i) {
    switch (pGeo->lefiGeometries::itemType(i)) {
      case lefiGeomLayerE:
        layerName = pGeo->lefiGeometries::getLayer(i);
        if (_cir.hasLayer(layerName)) {
          //assert(layerName != "m0" and layerName != "v0");
          layerIdx = _cir.name2LayerIdx(layerName);
          pin._minLayerIdx = std::min(pin._minLayerIdx, layerIdx);
          pin._maxLayerIdx = std::max(pin._maxLayerIdx, layerIdx);
        }
        else {
          //assert(false);
          layerIdx = -1;
        }
        break;
      case lefiGeomRectE:
        if (layerIdx != -1) {
          lefRect = pGeo->lefiGeometries::getRect(i);
          xl = toDBUnit(lefRect->xl) + originX;
          yl = toDBUnit(lefRect->yl) + originY;
          xh = toDBUnit(lefRect->xh) + originX;
          yh = toDBUnit(lefRect->yh) + originY;
          assert(xl <= xh and yl <= yh);
          pin._vvBoxIds[layerIdx].emplace_back(pin._vBoxes.size());
          pin._vBoxes.emplace_back(xl, yl, xh, yh);
          pin._vBoxLayerIds.emplace_back(layerIdx);
        }
        break;
      default:
        assert(false);
    }
  }
}

void LefReader::parseObstruction(const lefiObstruction* p)
{
  lefiGeometries* pGeo = p->lefiObstruction::geometries();
  parseObsGeometry(pGeo);

}

void LefReader::parseObsGeometry(const lefiGeometries* pGeo)
{
  Primitive& prim = *_cir._vpPrims.back();
  const Int originX = prim._originX;
  const Int originY = prim._originY;

  Int layerIdx = -1;
  String layerName;
  lefiGeomRect* lefRect = nullptr;
  Int xl = 0, yl = 0, xh = 0, yh = 0;

  for (Int i = 0; i < pGeo->lefiGeometries::numItems(); ++i) {
    switch (pGeo->lefiGeometries::itemType(i)) {
      case lefiGeomLayerE:
        layerName = pGeo->lefiGeometries::getLayer(i);
        if (_cir.hasLayer(layerName)) {
          //assert(layerName != "m0" and layerName != "v0");
          layerIdx = _cir.name2LayerIdx(layerName);
        }
        else {
          //assert(false);
          layerIdx = -1;
        }
        break;
      case lefiGeomRectE:
        if (layerIdx != -1) {
          lefRect = pGeo->lefiGeometries::getRect(i);
          xl = toDBUnit(lefRect->xl) + originX;
          yl = toDBUnit(lefRect->yl) + originY;
          xh = toDBUnit(lefRect->xh) + originX;
          yh = toDBUnit(lefRect->yh) + originY;
          assert(xl <= xh and yl <= yh);
          prim._vObs.emplace_back(Box<Int>(xl, yl, xh, yh), layerIdx);
        }
        break;
      default:
        assert(false);
    }
  }
}

void LefReader::parseVia(const lefiVia* p)
{
  const String& viaName = p->lefiVia::name();
  const Int idx = _cir._vpVias.size();
  _cir._mViaName2Idx.emplace(viaName, idx);
  _cir._vpVias.emplace_back(new Via());

  Via& via = *_cir._vpVias.back();

  via._name = viaName;
  via._idx = idx;

  if (p->lefiVia::hasDefault()) {
    via._type = ViaTypeE::def;
  }
  else if (p->lefiVia::hasGenerated()) {
    via._type = ViaTypeE::generated;
  }
  
  via._vvBoxIds.resize(_cir.numLayers());
  for (Int i = 0; i < p->lefiVia::numLayers(); ++i) {
    const String layerName = p->lefiVia::layerName(i);
    const Int layerIdx = _cir.layer(layerName).idx();
    
    via._minLayerIdx = std::min(via._minLayerIdx, layerIdx);
    via._maxLayerIdx = std::max(via._maxLayerIdx, layerIdx);

    for (Int j = 0; j < p->lefiVia::numRects(i); ++j) {
      const Int xl = toDBUnit(p->lefiVia::xl(i, j));
      const Int yl = toDBUnit(p->lefiVia::yl(i, j));
      const Int xh = toDBUnit(p->lefiVia::xh(i, j));
      const Int yh = toDBUnit(p->lefiVia::yh(i, j));
      via._vvBoxIds[layerIdx].emplace_back(via._vBoxes.size());
      via._vBoxes.emplace_back(xl, yl, xh, yh);
      via._vBoxLayerIds.emplace_back(layerIdx);
    }
  }

}

void LefReader::parseLef58Area(MetalLayer& layer, const String& s)
{
 // TODO
}

void LefReader::parseLef58Spacing(MetalLayer& layer, const String& s)
{
  std::istringstream sin(s);
  std::stringstream ss;
  String buf;
  Lef58SpacingType1 keyword1 = Lef58SpacingType1::undef;
  Lef58SpacingType2 keyword2 = Lef58SpacingType2::undef;

  while (sin >> buf) {
    if (buf == "SPACING") {
      ss.str("");
      ss << buf;
    }
    else if (buf == "EOLPERPENDICULAR") {
      keyword1 = Lef58SpacingType1::EOLPERPENDICULAR;
      ss << " " << buf;
    }
    else if (buf == "AREA") {
      keyword1 = Lef58SpacingType1::AREA;
      ss << " " << buf;
    }
    else if (buf == "LAYER") {
      keyword1 = Lef58SpacingType1::LAYER;
      ss << " " << buf;
    }
    else if (buf == "NOTCHLENGTH") {
      keyword1 = Lef58SpacingType1::NOTCHLENGTH;
      ss << " " << buf;
    }
    else if (buf == "NOTCHSPAN") {
      keyword1 = Lef58SpacingType1::NOTCHSPAN;
      ss << " " << buf;
    }
    else if (buf == "ENDOFLINE") {
      keyword1 = Lef58SpacingType1::ENDOFLINE;
      ss << " " << buf;
    }
    else if (buf == "CONVEXCORNERS") {
      keyword1 = Lef58SpacingType1::CONVEXCORNERS;
      ss << " " << buf;
    }
    else if (buf == "TOCONCAVECORNER") {
      keyword2 = Lef58SpacingType2::TOCONCAVECORNER;
      ss << " " << buf;
    }
    else if (buf == "TONOTCHLENGTH") {
      keyword2 = Lef58SpacingType2::TONOTCHLENGTH;
      ss << " " << buf;
    }
    else if (buf == ";") {
      switch (keyword1) {
        case Lef58SpacingType1::ENDOFLINE:
          switch (keyword2) {
            case Lef58SpacingType2::undef: // default lef58 end of line spacing
              parseLef58SpacingEol(layer, ss.str());
            case Lef58SpacingType2::TOCONCAVECORNER:
            case Lef58SpacingType2::TONOTCHLENGTH:
              break;
          }
          break;
        case Lef58SpacingType1::EOLPERPENDICULAR:
        case Lef58SpacingType1::AREA:
        case Lef58SpacingType1::LAYER:
        case Lef58SpacingType1::NOTCHLENGTH:
        case Lef58SpacingType1::NOTCHSPAN:
        case Lef58SpacingType1::CONVEXCORNERS:
        case Lef58SpacingType1::undef:
          break;
      }
      ss << " " << buf;
    }
    else {
      ss << " " << buf;
    }
  }

}

void LefReader::parseLef58SpacingEol(MetalLayer& layer, const String& s)
{
 // TODO
}

//void LefReader::parseLef58SpacingTable(MetalLayer& layer, const String& s)
//{

//}

//void LefReader::parseLef58SpacingTablePrl(MetalLayer& layer, const String& s)
//{

//}

void LefReader::parseLef58EolKeepOut(MetalLayer& layer, const String& s)
{
 // TODO
}

void LefReader::parseLef58MinStep(MetalLayer& layer, const String& s)
{
 // TODO
}

void LefReader::parseLef58WidthTable(MetalLayer& layer, const String& s)
{
 // TODO
}

Int LefReader::toDBUnit(const Real val) const
{
  //assert(_cir._physRes != 0);
  //return std::lround(val / _cir._physRes);
  return std::lround(val * _dbUnit);
}

Int LefReader::toDBUnit2d(const Real val) const
{
  //return std::lround(val / _cir._physRes / _cir._physRes);
  return std::lround(val * _dbUnit * _dbUnit);
}

void LefReader::parse(const Vector<String>& vFileNames)
{
  using namespace lef;
  // callbacks
  lefrSetWarningLogFunction(warningCB);
  lefrSetLogFunction(errorCB);
  lefrSetMallocFunction(mallocCB);
  lefrSetReallocFunction(reallocCB);
  lefrSetFreeFunction(freeCB);
  lefrSetLineNumberFunction(lineNumberCB);
  lefrSetUserData((void*)3);

  lefrSetVersionCbk(versionCB);
  lefrSetVersionStrCbk(versionStrCB);
  lefrSetBusBitCharsCbk(busBitCharsCB);
  lefrSetDividerCharCbk(dividerCB);
  lefrSetManufacturingCbk(manufacturingCB);
  lefrSetUnitsCbk(unitCB);

  lefrSetPropBeginCbk(propDefBeginCB);
  lefrSetPropCbk(propDefCB);
  lefrSetPropEndCbk(propDefEndCB);

  lefrSetLayerCbk(layerCB);

  lefrSetMacroBeginCbk(macroBeginCB);
  lefrSetMacroCbk(macroCB);
  lefrSetMacroClassTypeCbk(macroClassTypeCB);
  lefrSetMacroOriginCbk(macroOriginCB);
  lefrSetMacroSizeCbk(macroSizeCB);
  lefrSetMacroForeignCbk(macroForeignCB);
  lefrSetMacroSiteCbk(macroSiteCB);
  lefrSetMacroEndCbk(macroEndCB);
  lefrSetPinCbk(pinCB);
  lefrSetObstructionCbk(obstructionCB);
  lefrSetViaCbk(viaCB);
  
  lefrSetLibraryEndCbk(doneCB);

  // warnings
  lefrSetMacroWarnings(30);
  lefrSetPinWarnings(30);
  
  lefrSetRegisterUnusedCallbacks();
  lefrSetOpenLogFileAppend();

  pLef = this;

  FILE* fin;
  int userData = 0;

  for (size_t i = 0; i < vFileNames.size(); ++i) {
    const String& fileName = vFileNames.at(i);

    lefrInit();
    if (!(fin = fopen(fileName.c_str(), "r"))) {
      spdlog::error("Cannot open LEF file : {}\n", fileName);
      exit(0);
    }
    int result = lefrRead(fin, fileName.c_str(), (void*)(intptr_t)userData);
    if (result) {
      spdlog::error("Reader return bad status on {}\n", fileName);
      exit(0);
    }
    lefrPrintUnusedCallbacks(stderr);
    lefrReleaseNResetMemory();

    fclose(fin);
  }


  pLef = nullptr;
  // Release allocated singleton data.
  lefrClear();

}

PROJECT_NAMESPACE_END
