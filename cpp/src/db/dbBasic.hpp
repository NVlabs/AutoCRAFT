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

#include "global/global.hpp"
#include "util/util.hpp"
#include "geo/point3d.hpp"

PROJECT_NAMESPACE_START

////////////////////////////////  
//          Layer             //  
////////////////////////////////
enum class LayerTypeE : Byte {
  undef = 0,    // undefined
  metal,        // metal layer
  cut           // via cut layer
  //res,        // resistor layer
  //pin,        // pin layer (text)
  //pinpin,     // real pin layer#pin layer
  //prboundary, // boundary layer
  //base,       // base layers: NP, PP, nlv...
  //fill,       // metal fill layer
};
constexpr Array<const util::enumUtil::StrValPair<LayerTypeE>, 3> LayerTypeEStr{
    {{LayerTypeE::undef, "undef"},
     {LayerTypeE::metal, "metal"},
     {LayerTypeE::cut, "cut"}}};

////////////////////////////////  
//          Grid              //  
////////////////////////////////
enum class TrackUseE : Byte {
  undef = 0,
  signal,
  power,
  free
};
constexpr Array<const util::enumUtil::StrValPair<TrackUseE>, 4> TrackUseEStr{
    {{TrackUseE::undef, "undef"},
     {TrackUseE::signal, "signal"},
     {TrackUseE::power, "power"},
     {TrackUseE::free, "free"}}};

////////////////////////////////  
//          Pin               //  
////////////////////////////////
enum class PinDirE : Byte {
  undef = 0,
  input,
  output,
  inout,
  feedthru
};
constexpr Array<const util::enumUtil::StrValPair<PinDirE>, 5> PinDirEStr{
    {{PinDirE::undef, "undef"},
     {PinDirE::input, "input"},
     {PinDirE::output, "output"},
     {PinDirE::inout, "inout"},
     {PinDirE::feedthru, "feedthru"}}};

enum class PinUseE : Byte {
  undef = 0,
  signal,
  vdd,
  vss
};
constexpr Array<const util::enumUtil::StrValPair<PinUseE>, 4> PinUseEStr{
    {{PinUseE::undef, "undef"},
     {PinUseE::signal, "signal"},
     {PinUseE::vdd, "vdd"},
     {PinUseE::vss, "vss"}}};

enum class PinShapeE : Byte {
  undef = 0,
  abutment,
  ring,
  feedthru
};
constexpr Array<const util::enumUtil::StrValPair<PinShapeE>, 4> PinShapeEStr{
    {{PinShapeE::undef, "undef"},
     {PinShapeE::abutment, "abutment"},
     {PinShapeE::ring, "ring"},
     {PinShapeE::feedthru, "feedthru"}}};

enum class IOPinLocE : Byte {
  undef = 0,
  left,
  right,
  bottom,
  top
};
constexpr Array<const util::enumUtil::StrValPair<IOPinLocE>, 5> IOPinLocEStr{
    {{IOPinLocE::undef,  "undef"},
     {IOPinLocE::left,   "left"},
     {IOPinLocE::right,  "right"},
     {IOPinLocE::bottom, "bottom"},
     {IOPinLocE::top,    "top"}}};

////////////////////////////////  
//          Cell              //  
////////////////////////////////
enum class Orient2dE : Byte {
  n = 0,  // R0
  w,  // R90
  s,  // R180
  e,  // R270
  fn, // MY
  fw, // MX90
  fs, // MX
  fe, // MY90
  undef
};
constexpr Array<const util::enumUtil::StrValPair<Orient2dE>, 9> Orient2dEStr{
    {{Orient2dE::undef, "undef"},
     {Orient2dE::n, "n"},
     {Orient2dE::w, "w"},
     {Orient2dE::s, "s"},
     {Orient2dE::e, "e"},
     {Orient2dE::fn, "fn"},
     {Orient2dE::fw, "fw"},
     {Orient2dE::fs, "fs"},
     {Orient2dE::fe, "fe"}}};
constexpr Array<const util::enumUtil::StrValPair<Orient2dE>, 9> Orient2dEStr2{
    {{Orient2dE::undef, "undef"},
     {Orient2dE::n, "R0"},
     {Orient2dE::w, "R90"},
     {Orient2dE::s, "R180"},
     {Orient2dE::e, "R270"},
     {Orient2dE::fn, "MY"},
     {Orient2dE::fw, "MX90"},
     {Orient2dE::fs, "MX"},
     {Orient2dE::fe, "MY90"}}};

////////////////////////////////  
//          Via               //  
////////////////////////////////
enum class ViaTypeE : Byte {
  undef = 0,
  def, // default
  generated
};
constexpr Array<const util::enumUtil::StrValPair<ViaTypeE>, 3> ViaTypeEStr{
    {{ViaTypeE::undef, "undef"},
     {ViaTypeE::def, "def"},
     {ViaTypeE::generated, "generated"}}};


////////////////////////////////  
//          Net               //  
////////////////////////////////
enum class NetTypeE : Byte {
  undef = 0,
  signal,
  vdd,
  vss,
  clock,
  critical
};
constexpr Array<const util::enumUtil::StrValPair<NetTypeE>, 6> NetTypeEStr{
    {{NetTypeE::undef, "undef"},
     {NetTypeE::signal, "signal"},
     {NetTypeE::vdd, "vdd"},
     {NetTypeE::vss, "vss"},
     {NetTypeE::clock, "clock"},
     {NetTypeE::critical, "critical"}}};

enum class NetIOLocE : Byte {
  undef = 0,
  left,
  right,
  bottom,
  top
};
constexpr Array<const util::enumUtil::StrValPair<NetIOLocE>, 5> NetIOLocEStr{
    {{NetIOLocE::undef,  "undef"},
     {NetIOLocE::left,   "left"},
     {NetIOLocE::right,  "right"},
     {NetIOLocE::bottom, "bottom"},
     {NetIOLocE::top,    "top"}}};

////////////////////////////////  
//          Sym               //  
////////////////////////////////
enum class SymAxisE : Byte {
  undef = 0,
  ver,
  hor,
  either
};
constexpr Array<const util::enumUtil::StrValPair<SymAxisE>, 4> SymAxisEStr{
    {{SymAxisE::undef, "undef"},
     {SymAxisE::ver, "ver"},
     {SymAxisE::hor, "hor"},
     {SymAxisE::either, "either"}}};

enum class SymPartE : Byte {
  undef = 0,
  a,
  b,
  self
};
constexpr Array<const util::enumUtil::StrValPair<SymPartE>, 4> SymPartEStr{
    {{SymPartE::undef, "undef"},
     {SymPartE::a, "a"},
     {SymPartE::b, "b"},
     {SymPartE::self, "self"}}};

////////////////////////////////  
//          Array             //  
////////////////////////////////
enum class ArrayPatternE : Byte {
  undef = 0,
  id, // interdigitation
  cc, // common-controid
  cs // central symmetrical
};
constexpr Array<const util::enumUtil::StrValPair<ArrayPatternE>, 4> ArrayPatternEStr{
    {{ArrayPatternE::undef, "undef"},
     {ArrayPatternE::id, "id"},
     {ArrayPatternE::cc, "cc"},
     {ArrayPatternE::cs, "cs"}}};

enum class ArrayPartE : Byte {
  undef = 0,
  a,
  b
};
constexpr Array<const util::enumUtil::StrValPair<ArrayPartE>, 3> ArrayPartEStr{
    {{ArrayPartE::undef, "undef"},
     {ArrayPartE::a, "a"},
     {ArrayPartE::b, "b"}}};

////////////////////////////////  
//          Ext               //  
////////////////////////////////
enum class ExtTypeE : Byte {
  undef = 0,
  cell,
  region,
  array
};
constexpr Array<const util::enumUtil::StrValPair<ExtTypeE>, 4> ExtTypeEStr{
    {{ExtTypeE::undef, "undef"},
     {ExtTypeE::cell, "cell"},
     {ExtTypeE::region, "region"},
     {ExtTypeE::array, "array"}}};

////////////////////////////////  
//          Route             //  
////////////////////////////////
enum class Direction2dE: Byte {
  undef = 0,
  left,
  right,
  up,
  down
};
constexpr Array<const util::enumUtil::StrValPair<Direction2dE>, 5> Direction2dEStr{
    {{Direction2dE::undef,  "undef"},
     {Direction2dE::left,   "left"},
     {Direction2dE::right,  "right"},
     {Direction2dE::up,     "up"},
     {Direction2dE::down,   "down"}}};

enum class Direction3dE: Byte {
  undef = 0,
  left,
  right,
  up,
  down,
  via_up,
  via_down
};
constexpr Array<const util::enumUtil::StrValPair<Direction3dE>, 7> Direction3dEStr{
    {{Direction3dE::undef,  "undef"},
     {Direction3dE::left,   "left"},
     {Direction3dE::right,  "right"},
     {Direction3dE::up,     "up"},
     {Direction3dE::down,   "down"},
     {Direction3dE::via_up, "via_up"},
     {Direction3dE::down,   "via_down"}}};

enum class RoutePrefE : Byte {
  undef = 0,
  hor,
  ver
};
constexpr Array<const util::enumUtil::StrValPair<RoutePrefE>, 3> RoutePrefEStr{
    {{RoutePrefE::undef, "undef"},
     {RoutePrefE::hor, "hor"},
     {RoutePrefE::ver, "ver"}}};

enum class DrvE {
  undef,
  spc_prl,
  spc_eol,
  spc_cut,
  min_area,
  min_step,
  valid_width,
  valid_length
};

/////////////////////////////////////////
//           Funcs
/////////////////////////////////////////
namespace orient2d {
  extern Orient2dE rotate90(const Orient2dE o);
  extern Orient2dE rotate180(const Orient2dE o);
  extern Orient2dE rotate270(const Orient2dE o);
  extern Orient2dE flipX(const Orient2dE o);
  extern Orient2dE flipY(const Orient2dE o);
}

namespace direction3d {
  extern Direction3dE findDir(const Point3d<Int>& u, const Point3d<Int>& v);
}



PROJECT_NAMESPACE_END
