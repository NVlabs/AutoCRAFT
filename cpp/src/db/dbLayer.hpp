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

#include "dbBasic.hpp"
#include "ds/array2d.hpp"
#include "ds/hash.hpp"

PROJECT_NAMESPACE_START


class Layer {
  friend class Parser;
  friend class LefReader;

public:
  Layer()
    : _name(""), _idx(-1), _selfIdx(-1),
      _gdsLayerDrawing(0), _gdsDataTypeDrawing(0),
      _gdsLayerPin(0), _gdsDataTypePin(0),
      _gdsLayerColora(0), _gdsDataTypeColora(0),
      _gdsLayerColorb(0), _gdsDataTypeColorb(0)
  {}
  virtual ~Layer() {}

          const String& name()                const { return _name; }
          Int           idx()                 const { return _idx; }
          Int           selfIdx()             const { return _selfIdx; }

          Int           gdsLayerDrawing()     const { return _gdsLayerDrawing; }
          Int           gdsLayerPin()         const { return _gdsLayerPin; }
          Int           gdsLayerColora()      const { return _gdsLayerColora; }
          Int           gdsLayerColorb()      const { return _gdsLayerColorb; }

          Int           gdsDataTypeDrawing()  const { return _gdsDataTypeDrawing; }
          Int           gdsDataTypePin()      const { return _gdsDataTypePin; }
          Int           gdsDataTypeColora()   const { return _gdsDataTypeColora; }
          Int           gdsDataTypeColorb()   const { return _gdsDataTypeColorb; }


  virtual LayerTypeE    type()                const { return LayerTypeE::undef; }
  virtual String        typeStr()             const { return "undef"; }
  virtual bool          isMetal()             const { return false; }
  virtual bool          isCut()               const { return false; }

private:
  String _name;
  Int    _idx; // mi: 2i - 2; vi: 2i - 1
  Int    _selfIdx; // the idx in itself's group, mi: i - 1; vi: i - 1
  
  Int    _gdsLayerDrawing, _gdsDataTypeDrawing;
  Int    _gdsLayerPin, _gdsDataTypePin;
  Int    _gdsLayerColora, _gdsDataTypeColora;
  Int    _gdsLayerColorb, _gdsDataTypeColorb;
};

class MetalLayer : public Layer {
  friend class Parser;
  friend class LefReader;

public:
  MetalLayer()
    : 
      _direction(RoutePrefE::undef),
      _pitchX(0), _pitchY(0), _offsetX(0), _offsetY(0),  
      _width(0), _minWidth(0), _maxWidth(MAX_INT), _area(0),
      _spacing(0), _sameNetSpacing(0), _sameNetSpacingPG(0),
      _notchLength(0), _notchSpacing(0),
      _diagSpacing(0), _diagMinEdgeLength(0),
      _minStep(0), _maxEdges(0)
  {}
  ~MetalLayer() {}


  // vitual 
  LayerTypeE                      type()    const override { return LayerTypeE::metal; }
  String                          typeStr() const override { return "metal"; }
  bool                            isMetal() const override { return true; }

  RoutePrefE                      direction()                                 const { return _direction; }
  bool                            isVer()                                     const { return _direction == RoutePrefE::ver; }
  bool                            isHor()                                     const { return _direction == RoutePrefE::hor; }
  Int                             pitchX()                                    const { return _pitchX; }
  Int                             pitchY()                                    const { return _pitchY; }
  Int                             width()                                     const { return _width; }
  Int                             minWidth()                                  const { return _minWidth; }
  Int                             maxWidth()                                  const { return _maxWidth; }
  Int                             area()                                      const { return _area; }
  
  Int                             spacing()                                   const { return _spacing; }
  bool                            hasSpacing()                                const { return _spacing > 0; }
  
  Int                             sameNetSpacing()                            const { return _sameNetSpacing; }
  Int                             sameNetSpacingPG()                          const { return _sameNetSpacingPG; }
  bool                            hasSameNetSpacing()                         const { return _sameNetSpacing > 0; }
  bool                            hasSameNetSpacingPG()                       const { return _sameNetSpacingPG > 0; }
  
  Int                             notchLength()                               const { return _notchLength; }
  Int                             notchSpacing()                              const { return _notchSpacing; }
  bool                            hasNotchSpacing()                           const { return _notchSpacing > 0; }

  Int                             eolIdx(const Int eolWidth)                  const;
  Int                             eolSpacing(const Int i)                     const { return _vEolSpaces.at(i); }
  Int                             eolWidth(const Int i)                       const { return _vEolWidths.at(i); }
  Int                             eolWithin(const Int i)                      const { return _vEolWithins.at(i); }
  Int                             eolParSpace(const Int i)                    const { return _vEolParSpaces.at(i); }
  Int                             eolParWithin(const Int i)                   const { return _vEolParWithins.at(i); }
  bool                            eolHasTwoEdge(const Int i)                  const { return _vEolHasTwoEdges.at(i); }
  bool                            hasEolParellelEdge(const Int i)             const { return _vEolParSpaces.at(i) > 0; }
  bool                            hasEolSpacing()                             const { return _vEolSpaces.size() > 0; }

  Int                             diagSpacing()                               const { return _diagSpacing; }
  Int                             diagMinEdgeLength()                         const { return _diagMinEdgeLength; }
  bool                            hasDiagSpacing()                            const { return _diagSpacing > 0; }
  bool                            hasDiagMinEdgeLength()                      const { return _diagMinEdgeLength > 0; }
  
  Int                             prlSpacing(const Int width, const Int prl)  const;
  bool                            hasSpacingTable()                           const { return _vSpacingTablePrl.size() > 0; }

  Int                             minStep()                                   const { return _minStep; }
  Int                             maxEdges()                                  const { return _maxEdges; }

  // yaml rules
  Int                             validWidthIdx(const Int w)                  const;
  Int                             validWidth(const Int i)                     const { return _vValidWidths.at(i); }
  Int                             maxValidWidth()                             const { return _vValidWidths.back(); }
  Int                             width2ValidWidth(const Int w)               const { return w >= maxValidWidth() ? w : _vValidWidths.at(validWidthIdx(w)); }
  bool                            isValidWidth(const Int w)                   const { return std::binary_search(_vValidWidths.begin(), _vValidWidths.end(), w) or w > _vValidWidths.back(); }
  
  Int                             validLengthIdx(const Int l)                 const;
  Int                             validLength(const Int i)                    const { return _vValidLengths.at(i); }
  Int                             maxValidLength()                            const { return _vValidLengths.back(); }
  Int                             length2ValidLength(const Int l)             const { return l >= maxValidLength() ? l : _vValidLengths.at(validLengthIdx(l)); }
  bool                            isValidLength(const Int l)                  const { return std::binary_search(_vValidLengths.begin(), _vValidLengths.end(), l) or l > _vValidLengths.back(); }

  // rc data
  Real                            unitRL3Sigma(const Int w)                   const { return std::get<0>(_rTable.at(w)); }
  Real                            unitRMean(const Int w)                      const { return std::get<1>(_rTable.at(w)); }
  Real                            unitRU3Sigma(const Int w)                   const { return std::get<2>(_rTable.at(w)); }
  const Tuple<Real, Real, Real>&  unitR(const Int w)                          const { return _rTable.at(w); }
  Real                            unitCL3Sigma(const Int w, const Int s)      const { return std::get<0>(_cTable.at({w, s}).first); }
  Real                            unitCMean(const Int w, const Int s)         const { return std::get<1>(_cTable.at({w, s}).first); }
  Real                            unitCU3Sigma(const Int w, const Int s)      const { return std::get<2>(_cTable.at({w, s}).first); }
  const Tuple<Real, Real, Real>&  unitC(const Int w, const Int s)             const { return _cTable.at({w, s}).first; }
  Real                            unitCCL3Sigma(const Int w, const Int s)     const { return std::get<0>(_cTable.at({w, s}).second); }
  Real                            unitCCMean(const Int w, const Int s)        const { return std::get<1>(_cTable.at({w, s}).second); }
  Real                            unitCCU3Sigma(const Int w, const Int s)     const { return std::get<2>(_cTable.at({w, s}).second); }
  const Tuple<Real, Real, Real>&  unitCC(const Int w, const Int s)            const { return _cTable.at({w, s}).second; }

private:


  RoutePrefE    _direction;
  Int           _pitchX, _pitchY;
  Int           _offsetX, _offsetY;
  Int           _width; // default width
  Int           _minWidth, _maxWidth;
  Int           _area;
  Int           _spacing;
  Int           _sameNetSpacing;
  Int           _sameNetSpacingPG;
  Int           _notchLength, _notchSpacing;
  Int           _diagSpacing;
  Int           _diagMinEdgeLength;

  // eol spacing
  Vector<Int>   _vEolSpaces;
  Vector<Int>   _vEolWidths;
  Vector<Int>   _vEolWithins;
  Vector<Int>   _vEolParSpaces;
  Vector<Int>   _vEolParWithins;
  Vector<Byte>  _vEolHasTwoEdges;

  // spacing table rule
  Vector<Int>   _vSpacingTablePrl;
  Vector<Int>   _vSpacingTableWidth; 
  Array2d<Int>  _spacingTable;

  Int           _minStep;
  Int           _maxEdges;

  // yaml rules
  Vector<Int>   _vValidWidths;
  Vector<Int>   _vValidLengths;

  // rc data
  FlatHashMap<Int, Tuple<Real, Real, Real>> _rTable;
  FlatHashMap<Pair<Int, Int>, Pair<Tuple<Real, Real, Real>, Tuple<Real, Real, Real>>> _cTable;
};

class CutLayer : public Layer {
  friend class Parser;
  friend class LefReader;

public:
  CutLayer()
    : _spaceX(0), _spaceY(0), _widthX(0), _widthY(0),
      _vencAL(0), _vencAH(0), _vencPL(0), _vencPH(0),
      _rL3Sigma(0), _rMean(0), _rU3Sigma(0)
  {}
  ~CutLayer() {}
  
  LayerTypeE                      type()    const override { return LayerTypeE::cut; }
  String                          typeStr() const override { return "cut"; }
  bool                            isCut()   const override { return true; }
                                  
  Int                             spaceX()    const { return _spaceX; }
  Int                             spaceY()    const { return _spaceY; }
  Int                             widthX()    const { return _widthX; }
  Int                             widthY()    const { return _widthY; }
  Int                             vencAL()    const { return _vencAL; }
  Int                             vencAH()    const { return _vencAH; }
  Int                             vencPL()    const { return _vencPL; }
  Int                             vencPH()    const { return _vencPH; }

  Real                            rL3Sigma()  const { return _rL3Sigma; }
  Real                            rMean()     const { return _rMean; }
  Real                            rU3Sigma()  const { return _rU3Sigma; }

private:

  // yaml rules
  Int _spaceX, _spaceY;
  Int _widthX, _widthY;
  Int _vencAL, _vencAH; // along the trace-length
  Int _vencPL, _vencPH; // perpendular direction

  // r data
  Real _rL3Sigma, _rMean, _rU3Sigma; // resistance
};

PROJECT_NAMESPACE_END
