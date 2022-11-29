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

#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/operators.h>

#include "db/dbCir.hpp"
#include "geo/segment.hpp"
#include "io/parser.hpp"
#include "place/placeMgr.hpp"
#include "place/placeSMT.hpp"
#include "route/routeMgr.hpp"
#include "route/routeGrMgr.hpp"
#include "route/routeDrMgr.hpp"
#include "route/routeDrPS.hpp"
#include "route/routeDrPost.hpp"
#include "io/writer.hpp"
#include "ds/array2d.hpp"
#include "ds/array3d.hpp"
#include "rl/rlUtil.hpp"

namespace py = pybind11;

using PROJECT_NAMESPACE::Int;
using PROJECT_NAMESPACE::Real;
using PROJECT_NAMESPACE::Pair;
using PROJECT_NAMESPACE::Tuple;
using PROJECT_NAMESPACE::Vector;
using PROJECT_NAMESPACE::Point;
using PROJECT_NAMESPACE::Point3d;
using PROJECT_NAMESPACE::Segment;
using PROJECT_NAMESPACE::Segment3d;
using PROJECT_NAMESPACE::Box;
using PROJECT_NAMESPACE::TopoTree;
using PROJECT_NAMESPACE::Array2d;
using PROJECT_NAMESPACE::Array3d;

PYBIND11_MAKE_OPAQUE(Vector<int>);
PYBIND11_MAKE_OPAQUE(Vector<Vector<int>>);
PYBIND11_MAKE_OPAQUE(Vector<Vector<Vector<int>>>);
PYBIND11_MAKE_OPAQUE(Vector<Vector<Vector<Vector<int>>>>);

PYBIND11_MAKE_OPAQUE(Vector<double>);
PYBIND11_MAKE_OPAQUE(Vector<Vector<double>>);
PYBIND11_MAKE_OPAQUE(Vector<Vector<Vector<double>>>);
PYBIND11_MAKE_OPAQUE(Vector<Vector<Vector<Vector<double>>>>);

PYBIND11_MAKE_OPAQUE(Vector<Point<int>>);
PYBIND11_MAKE_OPAQUE(Vector<Point<double>>);
PYBIND11_MAKE_OPAQUE(Vector<Vector<Point<int>>>);
PYBIND11_MAKE_OPAQUE(Vector<Vector<Point<double>>>);

PYBIND11_MAKE_OPAQUE(Vector<Point3d<int>>);
PYBIND11_MAKE_OPAQUE(Vector<Point3d<double>>);
PYBIND11_MAKE_OPAQUE(Vector<Vector<Point3d<int>>>);
PYBIND11_MAKE_OPAQUE(Vector<Vector<Point3d<double>>>);

PYBIND11_MAKE_OPAQUE(Vector<Segment<int>>);
PYBIND11_MAKE_OPAQUE(Vector<Segment<double>>);
PYBIND11_MAKE_OPAQUE(Vector<Vector<Segment<int>>>);
PYBIND11_MAKE_OPAQUE(Vector<Vector<Segment<double>>>);

PYBIND11_MAKE_OPAQUE(Vector<Segment3d<int>>);
PYBIND11_MAKE_OPAQUE(Vector<Segment3d<double>>);
PYBIND11_MAKE_OPAQUE(Vector<Vector<Segment3d<int>>>);
PYBIND11_MAKE_OPAQUE(Vector<Vector<Segment3d<double>>>);

PYBIND11_MAKE_OPAQUE(Vector<TopoTree::SegData>);
PYBIND11_MAKE_OPAQUE(Vector<Vector<TopoTree::SegData>>);

PYBIND11_MAKE_OPAQUE(Vector<Box<int>>);
PYBIND11_MAKE_OPAQUE(Vector<Box<double>>);
PYBIND11_MAKE_OPAQUE(Vector<Vector<Box<int>>>);
PYBIND11_MAKE_OPAQUE(Vector<Vector<Box<double>>>);

PYBIND11_MAKE_OPAQUE(Vector<Pair<Box<int>, int>>);
PYBIND11_MAKE_OPAQUE(Vector<Vector<Pair<Box<int>, int>>>);

PYBIND11_MAKE_OPAQUE(Vector<Array2d<int>>);
PYBIND11_MAKE_OPAQUE(Vector<Array2d<double>>);
PYBIND11_MAKE_OPAQUE(Vector<Array3d<int>>);
PYBIND11_MAKE_OPAQUE(Vector<Array3d<double>>);

namespace PROJECT_NAMESPACE::api {

////////////////////////////////  
//          Basics            //  
////////////////////////////////
void initLayerTypeEnum(py::module& m)
{
  py::enum_<LayerTypeE>(m, "layer_type_e")
    .value("undef", LayerTypeE::undef)
    .value("metal", LayerTypeE::metal)
    .value("cut", LayerTypeE::cut);
}

void initTrackUseEnum(py::module& m)
{
  py::enum_<TrackUseE>(m, "track_use_e")
    .value("undef", TrackUseE::undef)
    .value("signal", TrackUseE::signal)
    .value("power", TrackUseE::power);
}

void initPinDirEnum(py::module& m)
{
  py::enum_<PinDirE>(m, "pin_dir_e")
    .value("undef", PinDirE::undef)
    .value("input", PinDirE::input)
    .value("output", PinDirE::output)
    .value("inout", PinDirE::inout)
    .value("feedthru", PinDirE::feedthru);
}

void initPinUseEnum(py::module& m)
{
  py::enum_<PinUseE>(m, "pin_use_e")
    .value("undef", PinUseE::undef)
    .value("signal", PinUseE::signal)
    .value("vdd", PinUseE::vdd)
    .value("vss", PinUseE::vss);
}

void initPinShapeEnum(py::module& m)
{
  py::enum_<PinShapeE>(m, "pin_shape_e")
    .value("undef", PinShapeE::undef)
    .value("abutment", PinShapeE::abutment)
    .value("ring", PinShapeE::ring)
    .value("feedthru", PinShapeE::feedthru);
}

void initIOPinLocEnum(py::module& m)
{
  py::enum_<IOPinLocE>(m, "io_pin_loc_e")
    .value("undef", IOPinLocE::undef)
    .value("left", IOPinLocE::left)
    .value("right", IOPinLocE::right)
    .value("bottom", IOPinLocE::bottom)
    .value("top", IOPinLocE::top);
}

void initOrient2dEnum(py::module& m)
{
  py::enum_<Orient2dE>(m, "orient2d_e")
    .value("undef", Orient2dE::undef)
    .value("n", Orient2dE::n)
    .value("w", Orient2dE::w)
    .value("s", Orient2dE::s)
    .value("e", Orient2dE::e)
    .value("fn", Orient2dE::fn)
    .value("fw", Orient2dE::fw)
    .value("fs", Orient2dE::fs)
    .value("fe", Orient2dE::fe);
}

void initViaTypeEnum(py::module& m)
{
  py::enum_<ViaTypeE>(m, "via_type_e")
    .value("undef", ViaTypeE::undef)
    .value("def", ViaTypeE::def)
    .value("generated", ViaTypeE::generated);
}


void initNetTypeEnum(py::module& m)
{
  py::enum_<NetTypeE>(m, "net_type_e")
    .value("undef", NetTypeE::undef)
    .value("signal", NetTypeE::signal)
    .value("vdd", NetTypeE::vdd)
    .value("vss", NetTypeE::vss)
    .value("clock", NetTypeE::clock)
    .value("critical", NetTypeE::critical);
}

void initNetIOLocEnum(py::module& m)
{
  py::enum_<NetIOLocE>(m, "net_io_loc_e")
    .value("undef", NetIOLocE::undef)
    .value("left", NetIOLocE::left)
    .value("right", NetIOLocE::right)
    .value("bottom", NetIOLocE::bottom)
    .value("top", NetIOLocE::top);
}

void initSymAxisEnum(py::module& m)
{
  py::enum_<SymAxisE>(m, "sym_axis_e")
    .value("undef", SymAxisE::undef)
    .value("ver", SymAxisE::ver)
    .value("hor", SymAxisE::hor)
    .value("either", SymAxisE::either);
}

void initSymPartEnum(py::module& m)
{
  py::enum_<SymPartE>(m, "sym_part_e")
    .value("undef", SymPartE::undef)
    .value("a", SymPartE::a)
    .value("b", SymPartE::b)
    .value("self", SymPartE::self);
}

void initArrayPatternEnum(py::module& m)
{
  py::enum_<ArrayPatternE>(m, "array_pattern_e")
    .value("undef", ArrayPatternE::undef)
    .value("id", ArrayPatternE::id)
    .value("cc", ArrayPatternE::cc)
    .value("cs", ArrayPatternE::cs);
}

void initArrayPartEnum(py::module& m)
{
  py::enum_<ArrayPartE>(m, "array_part_e")
    .value("undef", ArrayPartE::undef)
    .value("a", ArrayPartE::a)
    .value("b", ArrayPartE::b);
}

void initExtTypeEnum(py::module& m)
{
  py::enum_<ExtTypeE>(m, "ext_type_e")
    .value("undef", ExtTypeE::undef)
    .value("cell", ExtTypeE::cell)
    .value("region", ExtTypeE::region)
    .value("array", ExtTypeE::array);
}

void initDirection2dEnum(py::module& m)
{
  py::enum_<Direction2dE>(m, "direction2d_e")
    .value("undef", Direction2dE::undef)
    .value("left", Direction2dE::left)
    .value("right", Direction2dE::right)
    .value("up", Direction2dE::up)
    .value("down", Direction2dE::down);
}

void initDirection3dEnum(py::module& m)
{
  py::enum_<Direction3dE>(m, "direction3d_e")
    .value("undef", Direction3dE::undef)
    .value("left", Direction3dE::left)
    .value("right", Direction3dE::right)
    .value("up", Direction3dE::up)
    .value("down", Direction3dE::down)
    .value("via_up", Direction3dE::via_up)
    .value("via_down", Direction3dE::via_down);
}

void initRoutePrefEnum(py::module& m)
{
  py::enum_<RoutePrefE>(m, "route_pref_e")
    .value("undef", RoutePrefE::undef)
    .value("hor", RoutePrefE::hor)
    .value("ver", RoutePrefE::ver);
}

void initDrvEnum(py::module& m)
{
  py::enum_<DrvE>(m, "drv_e")
    .value("undef", DrvE::undef)
    .value("spc_prl", DrvE::spc_prl)
    .value("spc_eol", DrvE::spc_eol)
    .value("spc_cut", DrvE::spc_cut)
    .value("min_area", DrvE::min_area)
    .value("min_step", DrvE::min_step)
    .value("valid_width", DrvE::valid_width)
    .value("valid_length", DrvE::valid_length);
}


//////////////////////////////////
//          DB Api              //
//////////////////////////////////
void initLayerAPI(py::module& m)
{
  py::class_<Layer>(m, "layer")
    .def(py::init<>())
    .def("name", &Layer::name, py::return_value_policy::reference_internal)
    .def("idx", &Layer::idx)
    .def("self_idx", &Layer::selfIdx)
    .def("gds_layer_drawing", &Layer::gdsLayerDrawing)
    .def("gds_layer_pin", &Layer::gdsLayerPin)
    .def("gds_layer_color_a", &Layer::gdsLayerColora)
    .def("gds_layer_color_b", &Layer::gdsLayerColorb)
    .def("gds_data_type_drawing", &Layer::gdsDataTypeDrawing)
    .def("gds_data_type_pin", &Layer::gdsDataTypePin)
    .def("gds_data_type_color_a", &Layer::gdsDataTypeColora)
    .def("gds_data_type_color_b", &Layer::gdsDataTypeColorb)
    .def("type", &Layer::type)
    .def("type_str", &Layer::typeStr)
    .def("is_metal", &Layer::isMetal)
    .def("is_cut", &Layer::isCut);


  py::class_<MetalLayer, Layer>(m, "metal_layer")
    .def(py::init<>())
    .def("type", &MetalLayer::type)
    .def("type_str", &MetalLayer::typeStr)
    .def("is_metal", &MetalLayer::isMetal)
    .def("direction", &MetalLayer::direction)
    .def("is_ver", &MetalLayer::isVer)
    .def("is_hor", &MetalLayer::isHor)
    .def("pitch_x", &MetalLayer::pitchX)
    .def("pitch_y", &MetalLayer::pitchY)
    .def("width", &MetalLayer::width)
    .def("min_width", &MetalLayer::minWidth)
    .def("max_width", &MetalLayer::maxWidth)
    .def("area", &MetalLayer::area)
    .def("spacing", &MetalLayer::spacing)
    .def("has_spacing", &MetalLayer::hasSpacing)
    .def("same_net_spacing", &MetalLayer::sameNetSpacing)
    .def("has_same_net_spacing", &MetalLayer::hasSameNetSpacing)
    .def("same_net_spacing_pg", &MetalLayer::sameNetSpacingPG)
    .def("has_same_net_spacing_pg", &MetalLayer::hasSameNetSpacingPG)
    .def("notch_length", &MetalLayer::notchLength)
    .def("notch_spacing", &MetalLayer::notchSpacing)
    .def("has_notch_spacing", &MetalLayer::hasNotchSpacing)
    .def("eol_idx", &MetalLayer::eolIdx)
    .def("eol_spacing", &MetalLayer::eolSpacing)
    .def("eol_within", &MetalLayer::eolWithin)
    .def("eol_par_space", &MetalLayer::eolParSpace)
    .def("eol_par_within", &MetalLayer::eolParWithin)
    .def("eol_has_two_edge", &MetalLayer::eolHasTwoEdge)
    .def("has_eol_parallel_edge", &MetalLayer::hasEolParellelEdge)
    .def("has_eol_spacing", &MetalLayer::hasEolSpacing)
    .def("diag_spacing", &MetalLayer::diagSpacing)
    .def("diag_min_edge_length", &MetalLayer::diagMinEdgeLength)
    .def("has_diag_spacing", &MetalLayer::hasDiagSpacing)
    .def("has_diag_min_edge_length", &MetalLayer::hasDiagMinEdgeLength)
    .def("prl_spacing", &MetalLayer::prlSpacing)
    .def("has_spacing_table", &MetalLayer::hasSpacingTable)
    .def("min_step", &MetalLayer::minStep)
    .def("max_edges", &MetalLayer::maxEdges)
    .def("valid_width_idx", &MetalLayer::validWidthIdx)
    .def("valid_width", &MetalLayer::validWidth)
    .def("max_valid_width", &MetalLayer::maxValidWidth)
    .def("is_valid_width", &MetalLayer::isValidWidth)
    .def("valid_length_idx", &MetalLayer::validLengthIdx)
    .def("valid_length", &MetalLayer::validLength)
    .def("max_valid_length", &MetalLayer::maxValidLength)
    .def("to_valid_length", &MetalLayer::length2ValidLength)
    .def("is_valid_length", &MetalLayer::isValidLength)
    .def("unit_r_l3sigma", &MetalLayer::unitRL3Sigma)
    .def("unit_r_mean", &MetalLayer::unitRMean)
    .def("unit_r_u3sigma", &MetalLayer::unitRU3Sigma)
    .def("unit_r", &MetalLayer::unitR)
    .def("unit_c_l3sigma", &MetalLayer::unitCL3Sigma)
    .def("unit_c_mean", &MetalLayer::unitCMean)
    .def("unit_c_u3sigma", &MetalLayer::unitCU3Sigma)
    .def("unit_c", &MetalLayer::unitC)
    .def("unit_cc_l3sigma", &MetalLayer::unitCCL3Sigma)
    .def("unit_cc_mean", &MetalLayer::unitCCMean)
    .def("unit_cc_u3sigma", &MetalLayer::unitCCU3Sigma)
    .def("unit_cc", &MetalLayer::unitCC);

  
  py::class_<CutLayer, Layer>(m, "cut_layer")
    .def(py::init<>())
    .def("type", &CutLayer::type)
    .def("type_str", &CutLayer::typeStr)
    .def("is_cut", &CutLayer::isCut)
    .def("space_x", &CutLayer::spaceX)
    .def("space_y", &CutLayer::spaceY)
    .def("width_x", &CutLayer::widthX)
    .def("width_y", &CutLayer::widthY)
    .def("venc_al", &CutLayer::vencAL)
    .def("venc_ah", &CutLayer::vencAH)
    .def("venc_pl", &CutLayer::vencPL)
    .def("venc_ph", &CutLayer::vencPH)
    .def("r_l3sigma", &CutLayer::rL3Sigma)
    .def("r_mean", &CutLayer::rMean)
    .def("r_U3sigma", &CutLayer::rU3Sigma);
}

void initPlaceGridAPI(py::module& m)
{
  py::class_<PlaceGrid>(m, "place_grid")
    .def(py::init<>())
    .def("name", &PlaceGrid::name, py::return_value_policy::reference_internal)
    .def("origin", &PlaceGrid::origin)
    .def("origin_x", &PlaceGrid::originX)
    .def("origin_y", &PlaceGrid::originY)
    .def("step_x", &PlaceGrid::stepX)
    .def("step_y", &PlaceGrid::stepY)
    .def("floor_x", &PlaceGrid::floorX)
    .def("floor_y", &PlaceGrid::floorY)
    .def("ceil_x", &PlaceGrid::ceilX)
    .def("ceil_y", &PlaceGrid::ceilY)
    .def("is_on_grid_x", &PlaceGrid::isOnGridX)
    .def("is_on_grid_y", &PlaceGrid::isOnGridY)
    .def("is_on_grid", py::overload_cast<const Point<Int>&>(&PlaceGrid::isOnGrid, py::const_))
    .def("is_on_grid", py::overload_cast<const Box<Int>&>(&PlaceGrid::isOnGrid, py::const_));
}

void initRouteGridAPI(py::module& m)
{
  py::class_<RouteGrid>(m, "route_grid")
    .def(py::init<>())
    .def("name", &RouteGrid::name, py::return_value_policy::reference_internal)
    .def("origin", &RouteGrid::origin)
    .def("origin_x", &RouteGrid::originX)
    .def("origin_y", &RouteGrid::originY)
    .def("step_x", &RouteGrid::stepX)
    .def("step_y", &RouteGrid::stepY)
    .def("x_layer_idx", &RouteGrid::xLayerIdx)
    .def("y_layer_idx", &RouteGrid::yLayerIdx)
    .def("lower_layer_idx", &RouteGrid::lowerLayerIdx)
    .def("upper_layer_idx", &RouteGrid::upperLayerIdx)
    .def("cut_layer_idx", &RouteGrid::cutLayerIdx)
    .def("num_x_grids", &RouteGrid::numXGrids)
    .def("num_y_grids", &RouteGrid::numYGrids)
    .def("x_grid", &RouteGrid::xGrid)
    .def("y_grid", &RouteGrid::yGrid)
    .def("x_width", &RouteGrid::xWidth)
    .def("y_width", &RouteGrid::yWidth)
    .def("x_use", &RouteGrid::xUse)
    .def("y_use", &RouteGrid::yUse)
    .def("is_x_power_track", &RouteGrid::isXPowerTrack)
    .def("is_x_signal_track", &RouteGrid::isXSignalTrack)
    .def("is_x_free_track", &RouteGrid::isXFreeTrack)
    .def("is_y_power_track", &RouteGrid::isYPowerTrack)
    .def("is_y_signal_track", &RouteGrid::isYSignalTrack)
    .def("is_y_free_track", &RouteGrid::isYFreeTrack)
    .def("is_hor", &RouteGrid::isHor)
    .def("is_ver", &RouteGrid::isVer)
    .def("is_lower_layer_hor", &RouteGrid::isLowerLayerHor)
    .def("is_lower_layer_ver", &RouteGrid::isLowerLayerVer)
    .def("is_upper_layer_hor", &RouteGrid::isUpperLayerHor)
    .def("is_upper_layer_ver", &RouteGrid::isUpperLayerVer)
    .def("is_on_x_track", &RouteGrid::isOnXTrack)
    .def("is_on_y_track", &RouteGrid::isOnYTrack)
    .def("is_on_grid", py::overload_cast<const Int, const Int>(&RouteGrid::isOnGrid, py::const_))
    .def("is_on_grid", py::overload_cast<const Point<Int>&>(&RouteGrid::isOnGrid, py::const_))
    .def("via", &RouteGrid::via, py::return_value_policy::reference_internal)
    .def("prev_x", py::overload_cast<Int>(&RouteGrid::prevX, py::const_))
    .def("next_x", py::overload_cast<Int>(&RouteGrid::nextX, py::const_))
    .def("prev_y", py::overload_cast<Int>(&RouteGrid::prevY, py::const_))
    .def("next_y", py::overload_cast<Int>(&RouteGrid::nextY, py::const_))
    .def("prev_x", py::overload_cast<Int, const Int>(&RouteGrid::prevX, py::const_))
    .def("next_x", py::overload_cast<Int, const Int>(&RouteGrid::nextX, py::const_))
    .def("prev_y", py::overload_cast<Int, const Int>(&RouteGrid::prevY, py::const_))
    .def("next_y", py::overload_cast<Int, const Int>(&RouteGrid::nextY, py::const_))
    .def("prev_signal_x", py::overload_cast<Int, const Int>(&RouteGrid::prevSignalX, py::const_))
    .def("next_signal_x", py::overload_cast<Int, const Int>(&RouteGrid::nextSignalX, py::const_))
    .def("prev_signal_y", py::overload_cast<Int, const Int>(&RouteGrid::prevSignalY, py::const_))
    .def("next_signal_y", py::overload_cast<Int, const Int>(&RouteGrid::nextSignalY, py::const_))
    .def("floor_x", &RouteGrid::floorX)
    .def("floor_y", &RouteGrid::floorY)
    .def("ceil_x", &RouteGrid::ceilX)
    .def("ceil_y", &RouteGrid::ceilY)
    .def("x_loc_width", &RouteGrid::xLocWidth)
    .def("y_loc_width", &RouteGrid::yLocWidth)
    .def("wire_width", &RouteGrid::wireWidth)
    .def("is_x_loc_power_track", &RouteGrid::isXLocPowerTrack)
    .def("is_x_loc_signal_track", &RouteGrid::isXLocSignalTrack)
    .def("is_x_loc_free_track", &RouteGrid::isXLocFreeTrack)
    .def("is_y_loc_power_track", &RouteGrid::isYLocPowerTrack)
    .def("is_y_loc_signal_track", &RouteGrid::isYLocSignalTrack)
    .def("is_y_loc_free_track", &RouteGrid::isYLocFreeTrack)
    .def("has_valid_vias", &RouteGrid::hasValidVias);

}

void initPrimitiveAPI(py::module& m)
{
  py::class_<Primitive>(m, "primitive")
    .def(py::init<>())
    .def("name", &Primitive::name, py::return_value_policy::reference_internal)
    .def("idx", &Primitive::idx)
    .def("class_type", &Primitive::classType, py::return_value_policy::reference_internal)
    .def("origin_x", &Primitive::originX)
    .def("origin_y", &Primitive::originY)
    .def("size_x", &Primitive::sizeX)
    .def("size_y", &Primitive::sizeY)
    .def("area", &Primitive::area)
    .def("foreign_cell_name", &Primitive::foreignCellName, py::return_value_policy::reference_internal)
    .def("foreign_x", &Primitive::foreignX)
    .def("foreign_y", &Primitive::foreignY)
    .def("pin_const", py::overload_cast<const Int>(&Primitive::pin, py::const_), py::return_value_policy::reference_internal)
    .def("pin_const", py::overload_cast<const String&>(&Primitive::pin, py::const_), py::return_value_policy::reference_internal)
    .def("pin_name_to_idx", &Primitive::pinName2Idx)
    .def("v_pins", &Primitive::vPins)
    .def("num_pins", &Primitive::numPins)
    .def("obs", &Primitive::obs, py::return_value_policy::reference_internal)
    .def("v_obs", &Primitive::vObs, py::return_value_policy::reference_internal)
    .def("num_obs", &Primitive::numObs);

}

void initDevMapAPI(py::module& m)
{
  py::class_<DevMap>(m, "dev_map")
    .def(py::init<>())
    .def("name", &DevMap::name, py::return_value_policy::reference_internal)
    .def("prim", &DevMap::prim, py::return_value_policy::reference_internal)
    .def("multiplier", &DevMap::multiplier)
    .def("is_abutment", &DevMap::isAbutment)
    .def("dev_net", &DevMap::devNet, py::return_value_policy::reference_internal)
    .def("prim_net", &DevMap::primNet, py::return_value_policy::reference_internal)
    .def("dev_net_to_prim_net", &DevMap::devNet2PrimNet, py::return_value_policy::reference_internal)
    .def("prim_net_to_dev_net", &DevMap::primNet2DevNet, py::return_value_policy::reference_internal)
    .def("num_nets", &DevMap::numNets);
}

void initPinAPI(py::module& m)
{
  py::class_<Pin>(m, "pin")
    .def(py::init<>())
    .def("name", &Pin::name, py::return_value_policy::reference_internal)
    .def("dev_net_name", &Pin::devNetName, py::return_value_policy::reference_internal)
    .def("idx", &Pin::idx)
    .def("cell", py::overload_cast<>(&Pin::cell), py::return_value_policy::reference_internal)
    .def("cell_const", py::overload_cast<>(&Pin::cell, py::const_), py::return_value_policy::reference_internal)
    .def("net", py::overload_cast<>(&Pin::net), py::return_value_policy::reference_internal)
    .def("net_const", py::overload_cast<>(&Pin::net, py::const_), py::return_value_policy::reference_internal)
    .def("dir", &Pin::dir)
    .def("is_dir_input", &Pin::isDirInput)
    .def("is_dir_output", &Pin::isDirOutput)
    .def("is_dir_inout", &Pin::isDirInout)
    .def("is_dir_feedthru", &Pin::isDirFeedThru)
    .def("use", &Pin::use)
    .def("is_use_signal", &Pin::isUseSignal)
    .def("is_use_vdd", &Pin::isUseVdd)
    .def("is_use_vss", &Pin::isUseVss)
    .def("is_use_pg", &Pin::isUsePG)
    .def("shape", &Pin::shape)
    .def("is_shape_abutment", &Pin::isShapeAbutment)
    .def("is_shape_ring", &Pin::isShapeRing)
    .def("is_shape_feedthru", &Pin::isShapeFeedthru)
    .def("io_idx", &Pin::ioIdx)
    .def("box", py::overload_cast<const Int>(&Pin::box), py::return_value_policy::reference_internal)
    .def("box_const", py::overload_cast<const Int>(&Pin::box, py::const_), py::return_value_policy::reference_internal)
    .def("box", py::overload_cast<const Int, const Int>(&Pin::box), py::return_value_policy::reference_internal)
    .def("box_const", py::overload_cast<const Int, const Int>(&Pin::box, py::const_), py::return_value_policy::reference_internal)
    .def("v_boxes", py::overload_cast<>(&Pin::vBoxes), py::return_value_policy::reference_internal)
    .def("v_boxes_const", py::overload_cast<>(&Pin::vBoxes, py::const_), py::return_value_policy::reference_internal)
    .def("box_layer_idx", &Pin::boxLayerIdx)
    .def("num_boxes", py::overload_cast<>(&Pin::numBoxes, py::const_))
    .def("num_boxes", py::overload_cast<const Int>(&Pin::numBoxes, py::const_))
    .def("min_layer_idx", &Pin::minLayerIdx)
    .def("max_layer_idx", &Pin::maxLayerIdx)
    .def("v_acs", py::overload_cast<>(&Pin::vAcs), py::return_value_policy::reference_internal)
    .def("v_acs_const", py::overload_cast<>(&Pin::vAcs, py::const_), py::return_value_policy::reference_internal)
    .def("num_acs", &Pin::numAcs)
    .def("acs", py::overload_cast<const Int>(&Pin::acs), py::return_value_policy::reference_internal)
    .def("acs_const", py::overload_cast<const Int>(&Pin::acs, py::const_), py::return_value_policy::reference_internal)
    .def("super_acs", py::overload_cast<>(&Pin::superAcs), py::return_value_policy::reference_internal)
    .def("super_acs_const", py::overload_cast<>(&Pin::superAcs, py::const_), py::return_value_policy::reference_internal)
    .def("super_acs_idx", &Pin::superAcsIdx)
    .def("is_io", &Pin::isIO)
    .def("set_name", &Pin::setName)
    .def("set_net", &Pin::setNet)
    .def("set_super_axs_idx", &Pin::setSuperAcsIdx)
    .def("add_box", &Pin::addBox);
}

void initObsAPI(py::module& m)
{
  py::class_<Obs>(m, "obs")
    .def(py::init<>())
    .def("cell", py::overload_cast<>(&Obs::cell), py::return_value_policy::reference_internal)
    .def("cell_const", py::overload_cast<>(&Obs::cell, py::const_), py::return_value_policy::reference_internal)
    .def("box", py::overload_cast<>(&Obs::box), py::return_value_policy::reference_internal)
    .def("box_const", py::overload_cast<>(&Obs::box, py::const_), py::return_value_policy::reference_internal)
    .def("layer_idx", &Obs::layerIdx);
}

void initCellAPI(py::module& m)
{
  py::class_<Cell>(m, "cell")
    .def(py::init<>())
    //.def("cir", py::overload_cast<>(&Cell::cir), py::return_value_policy::reference_internal)
    //.def("cir_const", py::overload_cast<>(&Cell::cir, py::const_), py::return_value_policy::reference_internal)
    .def("name", &Cell::name, py::return_value_policy::reference_internal)
    .def("inst_name", &Cell::name, py::return_value_policy::reference_internal)
    .def("idx", &Cell::idx)
    .def("dev_map_const", &Cell::devMap, py::return_value_policy::reference_internal)
    .def("prim_const", &Cell::prim, py::return_value_policy::reference_internal)
    .def("pin", py::overload_cast<const Int>(&Cell::pin), py::return_value_policy::reference_internal)
    .def("pin_const", py::overload_cast<const Int>(&Cell::pin, py::const_), py::return_value_policy::reference_internal)
    .def("v_pins", py::overload_cast<>(&Cell::vPins), py::return_value_policy::reference_internal)
    .def("v_pins_const", py::overload_cast<>(&Cell::vPins, py::const_), py::return_value_policy::reference_internal)
    .def("num_pins", &Cell::numPins)
    .def("num_signal_pins", &Cell::numSignalPins, py::return_value_policy::reference_internal)
    .def("vdd_pin", py::overload_cast<>(&Cell::vddPin), py::return_value_policy::reference_internal)
    .def("vdd_pin_const", py::overload_cast<>(&Cell::vddPin, py::const_), py::return_value_policy::reference_internal)
    .def("vss_pin", py::overload_cast<>(&Cell::vssPin), py::return_value_policy::reference_internal)
    .def("vss_pin_const", py::overload_cast<>(&Cell::vssPin, py::const_), py::return_value_policy::reference_internal)
    .def("has_vdd_pin", &Cell::hasVddPin)
    .def("has_vss_pin", &Cell::hasVssPin)
    .def("obs", py::overload_cast<const Int>(&Cell::obs), py::return_value_policy::reference_internal)
    .def("obs_const", py::overload_cast<const Int>(&Cell::obs, py::const_), py::return_value_policy::reference_internal)
    .def("v_obs", py::overload_cast<>(&Cell::vObs), py::return_value_policy::reference_internal)
    .def("v_obs_const", py::overload_cast<>(&Cell::vObs, py::const_), py::return_value_policy::reference_internal)
    .def("num_obs", &Cell::numObs)
    .def("loc", &Cell::loc)
    .def("xl", &Cell::xl)
    .def("yl", &Cell::yl)
    .def("xh", &Cell::xh)
    .def("yh", &Cell::yh)
    .def("grid_loc", &Cell::gridLoc)
    .def("col", &Cell::col)
    .def("row", &Cell::row)
    .def("width", &Cell::width)
    .def("height", &Cell::height)
    .def("area", &Cell::area)
    .def("center", &Cell::center)
    .def("center_x", &Cell::centerX)
    .def("center_y", &Cell::centerY)
    .def("region", py::overload_cast<>(&Cell::region), py::return_value_policy::reference_internal)
    .def("region_const", py::overload_cast<>(&Cell::region, py::const_), py::return_value_policy::reference_internal)
    .def("power_group_idx", &Cell::powerGroupIdx)
    .def("pre_place_cstr_const", &Cell::prePlaceCstr, py::return_value_policy::reference_internal)
    .def("has_pre_place_cstr", &Cell::hasPrePlace)
    .def("place_sym_cstr_const", &Cell::placeSymCstr, py::return_value_policy::reference_internal)
    .def("num_place_sym_cstrs", &Cell::numPlaceSymCstrs)
    .def("has_sym_cstr", &Cell::hasSym)
    .def("place_array_cstr_const", &Cell::placeArrayCstr, py::return_value_policy::reference_internal)
    .def("num_place_array_cstr", &Cell::numPlaceArrayCstrs)
    .def("has_array_cstr", &Cell::hasArray)
    .def("place_cluster_cstr_const", &Cell::placeClusterCstr, py::return_value_policy::reference_internal)
    .def("num_place_cluster_cstrs", &Cell::numPlaceClusterCstrs)
    .def("has_cluster_cstr", &Cell::hasCluster)
    .def("place_order_cstr_const", &Cell::placeOrderCstr, py::return_value_policy::reference_internal)
    .def("num_place_order_cstrs", &Cell::numPlaceOrderCstrs)
    .def("has_order_cstr", &Cell::hasOrder)
    .def("place_align_cstr_const", &Cell::placeAlignCstr, py::return_value_policy::reference_internal)
    .def("num_place_align_cstrs", &Cell::numPlaceAlignCstrs)
    .def("has_align_cstr", &Cell::hasAlign)
    .def("place_disjoint_cstr_const", &Cell::placeDisjointCstr, py::return_value_policy::reference_internal)
    .def("num_place_disjoint_cstrs", &Cell::numPlaceDisjointCstrs)
    .def("has_disjoint_cstr", &Cell::hasDisjoint)
    .def("place_ext_cstr_const", &Cell::placeExtCstr, py::return_value_policy::reference_internal)
    .def("has_extension_cstr", &Cell::hasExtension)
    .def("place_row_cstr_const", &Cell::placeRowCstr, py::return_value_policy::reference_internal)
    .def("has_row_cstr", &Cell::hasRow)
    .def("place_edge_dist_cstr_const", &Cell::placeEdgeDistCstr, py::return_value_policy::reference_internal)
    .def("has_edge_dist_cstr", &Cell::hasEdgeDist)
    .def("orient", &Cell::orient)
    .def("set_name", &Cell::setName)
    .def("set_inst_name", &Cell::setInstName)
    .def("set_idx", &Cell::setIdx)
    .def("set_region", &Cell::setRegion)
    .def("set_dev_map", &Cell::setDevMap)
    .def("set_prim", &Cell::setPrim)
    .def("set_loc", &Cell::setLoc)
    .def("set_orient", &Cell::setOrient)
    .def("rotate_90", &Cell::rotate90)
    .def("rotate_180", &Cell::rotate180)
    .def("rotate_270", &Cell::rotate270)
    .def("flip_x", &Cell::flipX)
    .def("flip_y", &Cell::flipY)
    .def("set_power_group_idx", &Cell::setPowerGroupIdx);
}

void initViaAPI(py::module& m)
{
  py::class_<Via>(m, "via")
    .def(py::init<>())
    .def("name", &Via::name, py::return_value_policy::reference_internal)
    .def("idx", &Via::idx)
    .def("type", &Via::type)
    .def("is_default", &Via::isDefault)
    .def("is_generated", &Via::isGenerated)
    .def("box", py::overload_cast<const int>(&Via::box), py::return_value_policy::reference_internal)
    .def("box_const", py::overload_cast<const int>(&Via::box, py::const_), py::return_value_policy::reference_internal)
    .def("box", py::overload_cast<const int, const Int>(&Via::box), py::return_value_policy::reference_internal)
    .def("box_const", py::overload_cast<const int, const Int>(&Via::box, py::const_), py::return_value_policy::reference_internal)
    .def("v_boxes", py::overload_cast<>(&Via::vBoxes), py::return_value_policy::reference_internal)
    .def("v_boxes_const", py::overload_cast<>(&Via::vBoxes, py::const_), py::return_value_policy::reference_internal)
    .def("box_layer_idx", &Via::boxLayerIdx)
    .def("num_boxes", py::overload_cast<>(&Via::numBoxes, py::const_))
    .def("num_boxes", py::overload_cast<const Int>(&Via::numBoxes, py::const_))
    .def("min_layer_idx", &Via::minLayerIdx)
    .def("max_layer_idx", &Via::maxLayerIdx);

}

void initTopoTreeAPI(py::module& m)
{
  py::class_<TopoTree::SegData>(m, "seg_data")
    .def(py::init<>())
    .def(py::init<const Via*, Orient2dE, Int, Int>())
    .def("via_const", &TopoTree::SegData::via, py::return_value_policy::reference_internal)
    .def_readwrite("orient", &TopoTree::SegData::orient)
    .def_readwrite("width", &TopoTree::SegData::width)
    .def_readwrite("ext", &TopoTree::SegData::ext)
    .def_readwrite("ro_idx", &TopoTree::SegData::roIdx);


  py::class_<TopoTree>(m, "topo_tree")
    .def(py::init<>())
    .def("net", py::overload_cast<>(&TopoTree::net), py::return_value_policy::reference_internal)
    .def("net_const", py::overload_cast<>(&TopoTree::net, py::const_), py::return_value_policy::reference_internal)
    .def("ro", py::overload_cast<>(&TopoTree::ro), py::return_value_policy::reference_internal)
    .def("ro_const", py::overload_cast<>(&TopoTree::ro, py::const_), py::return_value_policy::reference_internal)
    .def("pin", py::overload_cast<const Int>(&TopoTree::pin), py::return_value_policy::reference_internal)
    .def("pin_const", py::overload_cast<const Int>(&TopoTree::pin, py::const_), py::return_value_policy::reference_internal)
    .def("num_pins", &TopoTree::numPins)
    .def("add_pin", &TopoTree::addPinPtr)
    .def("set_net", &TopoTree::setNetPtr)
    .def("set_ro", &TopoTree::setRoPtr)
    .def("has_segs", &TopoTree::hasSeg)
    .def("num_segs", &TopoTree::numSegs)
    .def("length", &TopoTree::length)
    .def("length_2d", &TopoTree::length2d)
    .def("bbox", py::overload_cast<>(&TopoTree::bbox), py::return_value_policy::reference_internal)
    .def("bbox_const", py::overload_cast<>(&TopoTree::bbox, py::const_), py::return_value_policy::reference_internal)
    .def("init", &TopoTree::init)
    .def("v_routes", py::overload_cast<Vector<Segment3d<Int>>&>(&TopoTree::vRoutes, py::const_))
    .def("v_routes", py::overload_cast<Vector<Segment3d<Int>>&, Vector<TopoTree::SegData>&>(&TopoTree::vRoutes, py::const_))
    .def("query_segs", &TopoTree::querySegs)
    .def("v_adj_segs", &TopoTree::vAdjSegs)
    .def("build_spatial_pins", &TopoTree::buildSpatialPins)
    .def("add_route", py::overload_cast<const Point3d<Int>&, const Point3d<Int>&, const TopoTree::SegData&>(&TopoTree::addRoute))
    .def("add_route", py::overload_cast<const Segment3d<Int>&, const TopoTree::SegData&>(&TopoTree::addRoute))
    .def("remove_route", py::overload_cast<const Point3d<Int>&, const Point3d<Int>&, const bool>(&TopoTree::removeRoute))
    .def("remove_route", py::overload_cast<const Segment3d<Int>&, const bool>(&TopoTree::removeRoute))
    .def("remove_floating", py::overload_cast<>(&TopoTree::removeFloating))
    .def("remove_redundant", py::overload_cast<>(&TopoTree::removeRedundant))
    .def("refine_dangling", py::overload_cast<const bool, const bool>(&TopoTree::refineDangling), py::arg("const bool") = true, py::arg("const bool") = true)
    .def("refine_self_loop", py::overload_cast<>(&TopoTree::refineSelfLoop))
    .def("is_connected", &TopoTree::isConnected)
    .def("num_comps", &TopoTree::numComps)
    .def("num_comp_pts", &TopoTree::numCompPts)
    .def("num_comp_segs", &TopoTree::numCompSegs)
    //.def("comp_pts", py::overload_cast<const Int>(&TopoTree::compPts))
    //.def("comp_pts", py::overload_cast<const Int>(&TopoTree::compPts, py::const_))
    .def("v_comp_segs", py::overload_cast<const Int, Vector<Segment3d<Int>>&>(&TopoTree::vCompSegs, py::const_))
    .def("v_comp_segs", py::overload_cast<const Int, Vector<Segment3d<Int>>&, Vector<TopoTree::SegData>&>(&TopoTree::vCompSegs, py::const_))
    .def("add_pin_acs", &TopoTree::addPinAcs)
    .def("group_comps", &TopoTree::groupComps)
    .def("show_comps", &TopoTree::showComps)
    .def("clear", &TopoTree::clear)
    .def("v_paths", py::overload_cast<const Point3d<Int>&, const Point3d<Int>&, Vector<Vector<Segment3d<Int>>>&>(&TopoTree::vPaths, py::const_))
    //.def("v_paths", py::overload_cast<const Point3d<Int>&, const Point3d<Int>&, Vector<Vector<Segment3d<Int>>>&, Vector<Vector<TopoTree::SegData>>&>(&TopoTree::vPaths, py::const_))
    .def("v_paths", py::overload_cast<const Vector<Point3d<Int>>&, const Vector<Point3d<Int>>&, Vector<Vector<Segment3d<Int>>>&>(&TopoTree::vPaths, py::const_))
    //.def("v_paths", py::overload_cast<const Vector<Point3d<Int>>&, const Vector<Point3d<Int>>&, Vector<Vector<Segment3d<Int>>>&, Vector<Vector<TopoTree::SegData>>&>(&TopoTree::vPaths, py::const_))
    .def("v_paths", py::overload_cast<const Pin&, const Pin&, Vector<Vector<Segment3d<Int>>>&>(&TopoTree::vPaths, py::const_));
    //.def("v_paths", py::overload_cast<const Pin&, const Pin&, Vector<Vector<Segment3d<Int>>>&, Vector<Vector<TopoTree::SegData>>&>(&TopoTree::vPaths, py::const_));
    
}

void initRoutableAPI(py::module& m)
{
  py::class_<Routable>(m, "routable")
    .def(py::init<>())
    .def("idx", &Routable::idx)
    .def("net", py::overload_cast<>(&Routable::net), py::return_value_policy::reference_internal)  
    .def("net_const", py::overload_cast<>(&Routable::net, py::const_), py::return_value_policy::reference_internal)
    .def("sym_net", py::overload_cast<>(&Routable::symNet), py::return_value_policy::reference_internal)
    .def("sym_net_const", py::overload_cast<>(&Routable::symNet, py::const_), py::return_value_policy::reference_internal)
    .def("pin", py::overload_cast<const Int>(&Routable::pin), py::return_value_policy::reference_internal)
    .def("pin_const", py::overload_cast<const Int>(&Routable::pin, py::const_), py::return_value_policy::reference_internal)
    .def("num_pins", &Routable::numPins)
    .def("routable_idx", &Routable::routableIdx)
    .def("v_routable_ids", py::overload_cast<>(&Routable::vRoutableIds), py::return_value_policy::reference_internal)
    .def("v_routable_ids_const", py::overload_cast<>(&Routable::vRoutableIds, py::const_), py::return_value_policy::reference_internal)
    .def("num_routables", &Routable::numRoutables)
    //.def("wire_idx", &Routable::wireIdx)
    //.def("v_wire_ids", py::overload_cast<>(&Routable::vWireIds))
    //.def("v_wire_ids", py::overload_cast<>(&Routable::vWireIds, py::const_))
    //.def("wires", py::overload_cast<>(&Routable::sWires))
    //.def("wires", py::overload_cast<>(&Routable::sWires, py::const_))
    .def("v_wires", &Routable::vWires)
    .def("num_wires", &Routable::numWires)
    .def("has_wire", &Routable::hasWire)
    .def("v_arcs", &Routable::vArcs)
    .def("num_arcs", &Routable::numArcs)
    .def("has_arc", &Routable::hasArc)
    .def("sym_net_routable_idx", &Routable::symNetRoutableIdx)
    .def("has_sym_routable", &Routable::hasSymRoutable)
    .def("is_self_sym", &Routable::isSelfSym)
    .def("is_routed", &Routable::isRouted)
    .def("topo", py::overload_cast<>(&Routable::topo), py::return_value_policy::reference_internal)
    .def("topo_const", py::overload_cast<>(&Routable::topo, py::const_), py::return_value_policy::reference_internal)
    .def("init", &Routable::init)
    .def("set_idx", &Routable::setIdx)
    .def("set_net", &Routable::setNet)
    .def("set_sym_net", &Routable::setSymNet)
    .def("set_sym_net_routable_idx", &Routable::setSymNetRoutableIdx)
    .def("set_self_sym", &Routable::setSelfSym)
    .def("set_routed", &Routable::setRouted)
    .def("add_pin", &Routable::addPin)
    .def("add_routable_idx", &Routable::addRoutableIdx)
    //.def("add_wire_idx", &Routable::addWireIdx)
    .def("add_wire", &Routable::addWire)
    .def("clear_routing", &Routable::clearRouting)
    .def("add_route", &Routable::addRoute)
    .def("remove_route", &Routable::removeRoute)
    .def("remove_floating_routes", py::overload_cast<>(&Routable::removeFloatingRoutes))
    .def("refine_dangling_routes", py::overload_cast<>(&Routable::refineDanglingRoutes));

}

void initNetAPI(py::module& m)
{
  py::class_<Net>(m, "net")
    .def(py::init<>())
    //.def("cir", py::overload_cast<>(&Net::cir), py::return_value_policy::reference)
    //.def("cir_const", py::overload_cast<>(&Net::cir, py::const_), py::return_value_policy::reference)
    .def("name", &Net::name, py::return_value_policy::reference_internal)
    .def("idx", &Net::idx)
    .def("pin", py::overload_cast<const Int>(&Net::pin), py::return_value_policy::reference_internal)
    .def("pin", py::overload_cast<const String&>(&Net::pin), py::return_value_policy::reference_internal)
    .def("pin_const", py::overload_cast<const Int>(&Net::pin, py::const_), py::return_value_policy::reference_internal)
    .def("pin_const", py::overload_cast<const String&>(&Net::pin, py::const_), py::return_value_policy::reference_internal)
    .def("v_pins", py::overload_cast<>(&Net::vpPins, py::const_), py::return_value_policy::reference_internal)
    .def("name_to_pin_idx", &Net::name2PinIdx)
    .def("num_pins", &Net::numPins)
    .def("has_pin", &Net::hasPin)
    .def("type", &Net::type)
    .def("is_power", &Net::isPower)
    .def("is_vdd", &Net::isVdd)
    .def("is_vss", &Net::isVss)
    .def("is_signal", &Net::isSignal)
    .def("is_clock", &Net::isClock)
    .def("is_critical", &Net::isCritical)
    .def("priority", &Net::priority)
    .def("is_layer_valid", &Net::isLayerValid)
    .def("hor_tracks", &Net::horTracks)
    .def("ver_tracks", &Net::verTracks)
    .def("layer_priority", &Net::layerPriority)
    .def("is_io", &Net::isIO)
    .def("io_loc", &Net::ioLoc)
    .def("io_route_grid", &Net::ioRouteGrid)
    .def("io_pin", py::overload_cast<>(&Net::ioPin), py::return_value_policy::reference_internal)
    .def("io_pin_const", py::overload_cast<>(&Net::ioPin, py::const_), py::return_value_policy::reference_internal)
    .def("bbox", py::overload_cast<>(&Net::bbox), py::return_value_policy::reference_internal)
    .def("bbox_const", py::overload_cast<>(&Net::bbox, py::const_), py::return_value_policy::reference_internal)
    .def("min_pin_layer_idx", &Net::minPinLayerIdx)
    .def("max_pin_layer_idx", &Net::maxPinLayerIdx)
    .def("wire_bbox", py::overload_cast<>(&Net::wireBbox), py::return_value_policy::reference_internal)
    .def("wire_bbox_const", py::overload_cast<>(&Net::wireBbox, py::const_), py::return_value_policy::reference_internal)
    .def("min_wire_layer_idx", &Net::minWireLayerIdx)
    .def("max_wire_layer_idx", &Net::maxWireLayerIdx)
    .def("num_arcs", &Net::numArcs)
    .def("has_arc", &Net::hasArc)
    .def("v_arcs", py::overload_cast<Vector<Net::Arc>&>(&Net::vArcs, py::const_))
    .def("v_arcs", py::overload_cast<Vector<Net::Arc>&, Vector<Net::ArcData>&>(&Net::vArcs, py::const_))
    .def("arc_via", &Net::arcVia, py::return_value_policy::reference_internal)
    .def("arc_via_orient", &Net::arcViaOrient)
    .def("arc_width", &Net::arcWidth)
    .def("arc_ext", &Net::arcExt)
    .def("arc_data", &Net::arcData, py::return_value_policy::reference_internal)
    //.def("wire", py::overload_cast<const Int>(&Net::wire))
    //.def("wire", py::overload_cast<const Int>(&Net::wire, py::const_))
    //.def("v_wires", py::overload_cast<>(&Net::vWires))
    //.def("v_wires", py::overload_cast<>(&Net::vWires, py::const_))
    //.def("wires", py::overload_cast<>(&Net::sWires))
    //.def("wires", py::overload_cast<>(&Net::sWires, py::const_))
    .def("v_wires", &Net::vWires)
    .def("num_wires", &Net::numWires)
    .def("has_wire", &Net::hasWire)
    //.def("patch_wire", py::overload_cast<const Int>(&Net::patchWire))
    //.def("patch_wire", py::overload_cast<const Int>(&Net::patchWire, py::const_))
    //.def("v_patch_wires", py::overload_cast<>(&Net::vPatchWires))
    //.def("v_patch_wires", py::overload_cast<>(&Net::vPatchWires, py::const_))
    //.def("patch_wires", py::overload_cast<>(&Net::sPatchWires))
    //.def("patch_wires", py::overload_cast<>(&Net::sPatchWires, py::const_))
    .def("v_patch_wires", &Net::vPatchWires)
    .def("num_patch_wires", &Net::numPatchWires)
    .def("has_patch_wire", &Net::hasPatchWire)
    .def("routable", py::overload_cast<const Int>(&Net::routable), py::return_value_policy::reference_internal)
    .def("routable_const", py::overload_cast<const Int>(&Net::routable, py::const_), py::return_value_policy::reference_internal)
    .def("v_routables", py::overload_cast<>(&Net::vRoutables), py::return_value_policy::reference_internal)
    .def("v_routables_const", py::overload_cast<>(&Net::vRoutables, py::const_), py::return_value_policy::reference_internal)
    .def("num_routables", &Net::numRoutables)
    .def("has_routable", &Net::hasRoutable)
    .def("routable_order", &Net::routableOrder)
    .def("v_routable_order", py::overload_cast<>(&Net::vRoutableOrder), py::return_value_policy::reference_internal)
    .def("v_routable_order_const", py::overload_cast<>(&Net::vRoutableOrder, py::const_), py::return_value_policy::reference_internal)
    .def("route_path_match_cstr", py::overload_cast<>(&Net::routePathMatchCstr), py::return_value_policy::reference_internal)
    .def("route_path_match_cstr_const", py::overload_cast<>(&Net::routePathMatchCstr, py::const_), py::return_value_policy::reference_internal)
    .def("has_path_match", &Net::hasPathMatch)
    .def("conn_src_pin", py::overload_cast<const Int>(&Net::connSrcPin), py::return_value_policy::reference_internal)
    .def("conn_src_pin_const", py::overload_cast<const Int>(&Net::connSrcPin, py::const_), py::return_value_policy::reference_internal)
    .def("conn_tar_pin", py::overload_cast<const Int>(&Net::connTarPin), py::return_value_policy::reference_internal)
    .def("conn_tar_pin_const", py::overload_cast<const Int>(&Net::connTarPin, py::const_), py::return_value_policy::reference_internal)
    .def("conn_cstr", py::overload_cast<const Int>(&Net::connCstr), py::return_value_policy::reference_internal)
    .def("conn_cstr_const", py::overload_cast<const Int>(&Net::connCstr, py::const_), py::return_value_policy::reference_internal)
    .def("conn", py::overload_cast<const Int>(&Net::conn), py::return_value_policy::reference_internal)
    .def("conn_const", py::overload_cast<const Int>(&Net::conn, py::const_), py::return_value_policy::reference_internal)
    .def("v_conns", py::overload_cast<>(&Net::vConns), py::return_value_policy::reference_internal)
    .def("num_conns", &Net::numConns)
    .def("has_path_match", &Net::hasPathMatch)
    .def("sym_net", py::overload_cast<>(&Net::symNet), py::return_value_policy::reference_internal)
    .def("sym_net_const", py::overload_cast<>(&Net::symNet, py::const_), py::return_value_policy::reference_internal)
    .def("has_sym_net", &Net::hasSymNet)
    .def("is_self_sym", &Net::isSelfSym)
    .def("sym_axis", &Net::symAxis)
    .def("sym_axis_x", &Net::symAxisX)
    .def("sym_axis_y", &Net::symAxisY)
    .def("is_ver_sym", &Net::isVerSym)
    .def("is_hor_sym", &Net::isHorSym)
    .def("is_routed", &Net::isRouted)
    .def("num_fails", &Net::numFails)
    .def("topo", py::overload_cast<>(&Net::topo), py::return_value_policy::reference_internal)
    .def("topo", py::overload_cast<>(&Net::topo, py::const_), py::return_value_policy::reference_internal)
    .def("init", &Net::init)
    .def("clear_routing", &Net::clearRouting)
    .def("add_route", &Net::addRoute)
    .def("remove_route", &Net::removeRoute)
    .def("remove_floating_routes", py::overload_cast<>(&Net::removeFloatingRoutes))
    .def("refine_dangling_routes", py::overload_cast<>(&Net::refineDanglingRoutes))
    .def("set_priority", &Net::setPriority)
    .def("set_type", &Net::setType)
    .def("set_sym_axis", &Net::setSymAxis)
    .def("set_sym_net", py::overload_cast<Net&>(&Net::setSymNet))
    .def("set_self_sym", &Net::setSelfSym)
    .def("set_sym_axis_x", &Net::setSymAxisX)
    .def("set_sym_axis_y", &Net::setSymAxisY)
    .def("reset_bbox", &Net::resetBbox)
    .def("set_bbox", &Net::setBbox)
    .def("set_min_pin_layer_idx", &Net::setMinPinLayerIdx)
    .def("set_max_pin_layer_idx", &Net::setMaxPinLayerIdx)
    .def("reset_wire_bbox", &Net::resetWireBbox)
    .def("set_wire_bbox", &Net::resetWireBbox)
    .def("update_wire_bbox", &Net::updateWireBbox)
    .def("set_min_wire_layer_idx", &Net::setMinWireLayerIdx)
    .def("set_max_wire_layer_idx", &Net::setMaxWireLayerIdx)
    .def("set_routed", &Net::setRouted)
    .def("add_fail", &Net::addFail)
    .def("add_pin", py::overload_cast<Pin&>(&Net::addPin));
    //.def("clear_rl_pins", &Net::clearRLPins);

}

void initRegionAPI(py::module& m)
{
  py::class_<Region>(m, "region")
    .def(py::init<>())
    //.def("cir", py::overload_cast<>(&Region::cir), py::return_value_policy::reference)
    //.def("cir_const", py::overload_cast<>(&Region::cir, py::const_), py::return_value_policy::reference)
    .def("name", &Region::name, py::return_value_policy::reference_internal)
    .def("idx", &Region::idx)
    .def("place_grid_const", &Region::placeGrid, py::return_value_policy::reference_internal)
    .def("route_grid_const", &Region::routeGrid, py::return_value_policy::reference_internal)
    .def("num_route_grids", &Region::numRouteGrids)
    .def("power_route_grid_const", &Region::powerRouteGrid, py::return_value_policy::reference_internal)
    .def("num_power_route_grids", &Region::numPowerRouteGrids)
    .def("cell", py::overload_cast<const Int>(&Region::cell), py::return_value_policy::reference_internal)
    .def("cell_const", py::overload_cast<const Int>(&Region::cell, py::const_), py::return_value_policy::reference_internal)
    .def("num_cells", &Region::numCells)
    .def("edge_prim_const", &Region::edgePrim, py::return_value_policy::reference_internal)
    .def("edge_prim_east_const", &Region::edgePrimEast, py::return_value_policy::reference_internal)
    .def("edge_prim_west_const", &Region::edgePrimWest, py::return_value_policy::reference_internal)
    .def("edge_prim_south_const", &Region::edgePrimSouth, py::return_value_policy::reference_internal)
    .def("edge_prim_north_const", &Region::edgePrimNorth, py::return_value_policy::reference_internal)
    .def("edge_prim_north_east_const", &Region::edgePrimNorthEast, py::return_value_policy::reference_internal)
    .def("edge_prim_north_west_const", &Region::edgePrimNorthWest, py::return_value_policy::reference_internal)
    .def("edge_prim_south_east_const", &Region::edgePrimSouthEast, py::return_value_policy::reference_internal)
    .def("edge_prim_south_west_const", &Region::edgePrimSouthWest, py::return_value_policy::reference_internal)
    .def("edge_prim_orient", &Region::edgePrimOrient)
    .def("edge_prim_east_orient", &Region::edgePrimEastOrient)
    .def("edge_prim_west_orient", &Region::edgePrimWestOrient)
    .def("edge_prim_south_orient", &Region::edgePrimSouthOrient)
    .def("edge_prim_north_orient", &Region::edgePrimNorthOrient)
    .def("edge_prim_north_east_orient", &Region::edgePrimNorthEastOrient)
    .def("edge_prim_north_west_orient", &Region::edgePrimNorthWestOrient)
    .def("edge_prim_south_east_orient", &Region::edgePrimSouthEastOrient)
    .def("edge_prim_south_west_orient", &Region::edgePrimSouthWestOrient)
    .def("dummy_prim", &Region::dummyPrim, py::return_value_policy::reference_internal)
    .def("num_dummy_prims", &Region::numDummyPrims)
    .def("net", py::overload_cast<const Int>(&Region::net), py::return_value_policy::reference_internal)
    .def("net_const", py::overload_cast<const Int>(&Region::net, py::const_), py::return_value_policy::reference_internal)
    .def("num_nets", &Region::numNets)
    .def("num_signal_nets", &Region::numSignalNets)
    .def("power_net", py::overload_cast<const Int>(&Region::powerNet), py::return_value_policy::reference_internal)
    .def("power_net_const", py::overload_cast<const Int>(&Region::powerNet, py::const_), py::return_value_policy::reference_internal)
    .def("num_power_nets", &Region::numPowerNets)
    //.def("pre_place_cstr_const", &Region::prePlaceCstr, py::return_value_policy::reference_internal)
    //.def("num_pre_place_cstrs", &Region::numPrePlaceCstrs)
    .def("place_sym_cstr_const", &Region::placeSymCstr, py::return_value_policy::reference_internal)
    .def("num_place_sym_cstrs", &Region::numPlaceSymCstrs)
    .def("place_array_cstr_const", &Region::placeArrayCstr, py::return_value_policy::reference_internal)
    .def("num_place_array_cstrs", &Region::numPlaceArrayCstrs)
    .def("place_ext_cstr_const", &Region::placeExtCstr, py::return_value_policy::reference_internal)
    .def("num_place_ext_cstrs", &Region::numPlaceExtCstrs)
    .def("place_edge_dist_cstr_const", &Region::placeEdgeDistCstr, py::return_value_policy::reference_internal)
    .def("num_place_edge_dist_cstrs", &Region::numPlaceEdgeDistCstrs)
    .def("route_sym_cstr_const", &Region::routeSymCstr, py::return_value_policy::reference_internal)
    .def("num_route_sym_cstrs", &Region::numRouteSymCstrs)
    .def("net_prio_cstr_const", &Region::netPrioCstr, py::return_value_policy::reference_internal)
    .def("num_net_prio_cstrs", &Region::numNetPrioCstrs)
    .def("bbox_inner", py::overload_cast<>(&Region::bboxInner), py::return_value_policy::reference_internal)
    .def("bbox_inner_const", py::overload_cast<>(&Region::bboxInner, py::const_), py::return_value_policy::reference_internal)
    .def("bbox_outer", py::overload_cast<>(&Region::bboxOuter), py::return_value_policy::reference_internal)
    .def("bbox_outer_const", py::overload_cast<>(&Region::bboxOuter, py::const_), py::return_value_policy::reference_internal)
    .def("num_cell_groups", &Region::numCellGroups)
    .def("v_cell_group_ids", &Region::vCellGroupIds)
    .def("vv_cell_group_ids", &Region::vvCellGroupIds)
    .def("edge_cell", py::overload_cast<const Int>(&Region::edgeCell), py::return_value_policy::reference_internal)
    .def("edge_cell_const", py::overload_cast<const Int>(&Region::edgeCell, py::const_), py::return_value_policy::reference_internal)
    .def("v_edge_cells", py::overload_cast<>(&Region::vEdgeCells), py::return_value_policy::reference_internal)
    .def("v_edge_cells_const", py::overload_cast<>(&Region::vEdgeCells, py::const_), py::return_value_policy::reference_internal)
    .def("num_edge_cells", &Region::numEdgeCells)
    .def("dummy_cell", py::overload_cast<const Int>(&Region::dummyCell), py::return_value_policy::reference_internal)
    .def("dummy_cell_const", py::overload_cast<const Int>(&Region::dummyCell, py::const_), py::return_value_policy::reference_internal)
    .def("v_dummy_cells", py::overload_cast<>(&Region::vDummyCells), py::return_value_policy::reference_internal)
    .def("v_dummy_cells", py::overload_cast<>(&Region::vDummyCells, py::const_), py::return_value_policy::reference_internal)
    .def("num_dummy_cells", &Region::numDummyCells)
    .def("set_bbox_inner", &Region::setBboxInner)
    .def("set_bbox_outer", &Region::setBboxOuter)
    .def("add_edge_cell", &Region::addEdgeCell)
    .def("add_dummy_cell", &Region::addDummyCell);
}

void initPrePlaceCstrAPI(py::module& m)
{
  py::class_<PrePlaceCstr>(m, "pre_place_cstr")
    .def(py::init<>())
    .def("name", &PrePlaceCstr::name, py::return_value_policy::reference_internal)
    .def("idx", &PrePlaceCstr::idx)
    .def("cell", py::overload_cast<const Int>(&PrePlaceCstr::cell), py::return_value_policy::reference_internal)
    .def("cell_const", py::overload_cast<const Int>(&PrePlaceCstr::cell, py::const_), py::return_value_policy::reference_internal)
    .def("num_cells", &PrePlaceCstr::numCells)
    .def("cell_loc", &PrePlaceCstr::cellLoc)
    .def("has_cell_loc", &PrePlaceCstr::hasCellLoc)
    .def("cell_orient", &PrePlaceCstr::cellOrient)
    .def("has_cell_orient", &PrePlaceCstr::hasCellOrient)
    .def("region", py::overload_cast<const Int>(&PrePlaceCstr::region), py::return_value_policy::reference_internal)
    .def("region_const", py::overload_cast<const Int>(&PrePlaceCstr::region, py::const_), py::return_value_policy::reference_internal)
    .def("num_regions", &PrePlaceCstr::numRegions)
    .def("reg_loc", &PrePlaceCstr::regLoc)
    .def("has_reg_loc", &PrePlaceCstr::hasRegLoc);
}

void initPlaceSymCstrAPI(py::module& m)
{
  py::class_<PlaceSymCstr>(m, "place_sym_cstr")
    .def(py::init<>())
    .def("name", &PlaceSymCstr::name, py::return_value_policy::reference_internal)
    .def("idx", &PlaceSymCstr::idx)
    .def("axis", &PlaceSymCstr::axis)
    .def("is_hor", &PlaceSymCstr::isHor)
    .def("is_ver", &PlaceSymCstr::isVer)
    .def("cell", py::overload_cast<const Int>(&PlaceSymCstr::cell), py::return_value_policy::reference_internal)
    .def("cell_const", py::overload_cast<const Int>(&PlaceSymCstr::cell, py::const_), py::return_value_policy::reference_internal)
    .def("num_cells", &PlaceSymCstr::numCells)
    .def("sym_part", py::overload_cast<const Int>(&PlaceSymCstr::symPart, py::const_))
    .def("sym_part", py::overload_cast<const Cell&>(&PlaceSymCstr::symPart, py::const_))
    .def("is_part_a", py::overload_cast<const Int>(&PlaceSymCstr::isPartA, py::const_))
    .def("is_part_a", py::overload_cast<const Cell&>(&PlaceSymCstr::isPartA, py::const_))
    .def("is_part_b", py::overload_cast<const Int>(&PlaceSymCstr::isPartB, py::const_))
    .def("is_part_b", py::overload_cast<const Cell&>(&PlaceSymCstr::isPartB, py::const_))
    .def("is_self_sym", py::overload_cast<const Int>(&PlaceSymCstr::isSelfSym, py::const_))
    .def("is_self_sym", py::overload_cast<const Cell&>(&PlaceSymCstr::isSelfSym, py::const_))
    .def("num_self_syms", &PlaceSymCstr::numSelfSyms)
    .def("num_part_as", &PlaceSymCstr::numPartAs)
    .def("has_self_sym", &PlaceSymCstr::hasSelfSym)
    .def("cell_idx_to_idx", &PlaceSymCstr::cellIdx2Idx)
    .def("cell_idx_to_sym_idx", &PlaceSymCstr::cellIdx2SymIdx)
    .def("has_cell", &PlaceSymCstr::hasCell)
    .def("sym_cell", py::overload_cast<const Int>(&PlaceSymCstr::symCell), py::return_value_policy::reference_internal)
    .def("sym_cell", py::overload_cast<const Cell&>(&PlaceSymCstr::symCell), py::return_value_policy::reference_internal)
    .def("sym_cell_const", py::overload_cast<const Int>(&PlaceSymCstr::symCell, py::const_), py::return_value_policy::reference_internal)
    .def("sym_cell_const", py::overload_cast<const Cell&>(&PlaceSymCstr::symCell, py::const_), py::return_value_policy::reference_internal)
    .def("region", &PlaceSymCstr::region, py::return_value_policy::reference_internal);
}

void initPlaceArrayCstrAPI(py::module& m)
{
  py::class_<PlaceArrayCstr>(m, "place_array_cstr")
    .def(py::init<>())
    .def("name", &PlaceArrayCstr::name, py::return_value_policy::reference_internal)
    .def("idx", &PlaceArrayCstr::idx)
    .def("col", &PlaceArrayCstr::col)
    .def("row", &PlaceArrayCstr::row)
    .def("has_col", &PlaceArrayCstr::hasCol)
    .def("has_row", &PlaceArrayCstr::hasRow)
    .def("size", &PlaceArrayCstr::size)
    .def("has_size", &PlaceArrayCstr::hasSize)
    .def("use_auto_size", &PlaceArrayCstr::useAutoSize)
    .def("pattern", &PlaceArrayCstr::pattern)
    .def("is_id", &PlaceArrayCstr::isID)
    .def("is_cc", &PlaceArrayCstr::isCC)
    .def("is_cs", &PlaceArrayCstr::isCS)
    .def("has_pattern", &PlaceArrayCstr::hasPattern)
    .def("cell", py::overload_cast<const Int>(&PlaceArrayCstr::cell), py::return_value_policy::reference_internal)
    .def("cell_const", py::overload_cast<const Int>(&PlaceArrayCstr::cell, py::const_), py::return_value_policy::reference_internal)
    .def("num_cells", &PlaceArrayCstr::numCells)
    .def("array_part", py::overload_cast<const Int>(&PlaceArrayCstr::arrayPart, py::const_))
    .def("array_part", py::overload_cast<const Cell&>(&PlaceArrayCstr::arrayPart, py::const_))
    .def("is_part_a", py::overload_cast<const Int>(&PlaceArrayCstr::isPartA, py::const_))
    .def("is_part_a", py::overload_cast<const Cell&>(&PlaceArrayCstr::isPartA, py::const_))
    .def("is_part_b", py::overload_cast<const Int>(&PlaceArrayCstr::isPartB, py::const_))
    .def("is_part_b", py::overload_cast<const Cell&>(&PlaceArrayCstr::isPartB, py::const_))
    .def("cell_idx_to_idx", &PlaceArrayCstr::cellIdx2Idx)
    .def("has_cell", &PlaceArrayCstr::hasCell)
    .def("place_array_cstr", py::overload_cast<const Int>(&PlaceArrayCstr::placeArrayCstr), py::return_value_policy::reference_internal)
    .def("place_array_cstr_const", py::overload_cast<const Int>(&PlaceArrayCstr::placeArrayCstr, py::const_), py::return_value_policy::reference_internal)
    .def("num_place_array_cstrs", &PlaceArrayCstr::numPlaceArrayCstrs)
    .def("array_idx_to_idx", &PlaceArrayCstr::arrayIdx2Idx)
    .def("has_array", &PlaceArrayCstr::hasArray)
    .def("region", &PlaceArrayCstr::region, py::return_value_policy::reference_internal);
}

void initPlaceClusterCstrAPI(py::module& m)
{
  py::class_<PlaceClusterCstr>(m, "place_cluster_cstr")
    .def(py::init<>())
    .def("name", &PlaceClusterCstr::name, py::return_value_policy::reference_internal)
    .def("idx", &PlaceClusterCstr::idx)
    .def("cell", py::overload_cast<const Int>(&PlaceClusterCstr::cell), py::return_value_policy::reference_internal)
    .def("cell", py::overload_cast<const Int>(&PlaceClusterCstr::cell, py::const_), py::return_value_policy::reference_internal)
    .def("num_cells", &PlaceClusterCstr::numCells)
    .def("weight", &PlaceClusterCstr::weight);
}

void initPlaceExtCstrAPI(py::module& m)
{
  py::class_<PlaceExtCstr>(m, "place_ext_cstr")
    .def(py::init<>())
    .def("name", &PlaceExtCstr::name, py::return_value_policy::reference_internal)
    .def("idx", &PlaceExtCstr::idx)
    .def("type", &PlaceExtCstr::type)
    .def("is_cell", &PlaceExtCstr::isCell)
    .def("is_region", &PlaceExtCstr::isRegion)
    .def("is_array", &PlaceExtCstr::isArray)
    .def("cell", py::overload_cast<const Int>(&PlaceExtCstr::cell), py::return_value_policy::reference_internal)
    .def("cell_const", py::overload_cast<const Int>(&PlaceExtCstr::cell, py::const_), py::return_value_policy::reference_internal)
    .def("num_cells", &PlaceExtCstr::numCells)
    .def("region", py::overload_cast<const Int>(&PlaceExtCstr::region), py::return_value_policy::reference_internal)
    .def("region_const", py::overload_cast<const Int>(&PlaceExtCstr::region, py::const_), py::return_value_policy::reference_internal)
    .def("num_region", &PlaceExtCstr::numRegions)
    .def("place_array_cstr", py::overload_cast<const Int>(&PlaceExtCstr::placeArrayCstr), py::return_value_policy::reference_internal)
    .def("place_array_cstr_const", py::overload_cast<const Int>(&PlaceExtCstr::placeArrayCstr, py::const_), py::return_value_policy::reference_internal)
    .def("num_place_array_cstrs", &PlaceExtCstr::numPlaceArrayCstrs)
    .def("left_ext_idx", &PlaceExtCstr::leftExtIdx)
    .def("right_ext_idx", &PlaceExtCstr::rightExtIdx)
    .def("bottom_ext_idx", &PlaceExtCstr::bottomExtIdx)
    .def("top_ext_idx", &PlaceExtCstr::topExtIdx);
}

void initPlaceEdgeDistCstrAPI(py::module& m)
{
  py::class_<PlaceEdgeDistCstr>(m, "place_edge_dist_cstr")
    .def(py::init<>())
    .def("name", &PlaceEdgeDistCstr::name, py::return_value_policy::reference_internal)
    .def("idx", &PlaceEdgeDistCstr::idx)
    .def("cell", py::overload_cast<const Int>(&PlaceEdgeDistCstr::cell), py::return_value_policy::reference_internal)
    .def("cell_const", py::overload_cast<const Int>(&PlaceEdgeDistCstr::cell, py::const_), py::return_value_policy::reference_internal)
    .def("num_cells", &PlaceEdgeDistCstr::numCells)
    .def("dist_l", &PlaceEdgeDistCstr::distL)
    .def("dist_r", &PlaceEdgeDistCstr::distR)
    .def("dist_b", &PlaceEdgeDistCstr::distB)
    .def("dist_t", &PlaceEdgeDistCstr::distT)
    .def("region", &PlaceEdgeDistCstr::region, py::return_value_policy::reference_internal);
}

void initPlaceOrderCstrAPI(py::module& m)
{
  py::class_<PlaceOrderCstr>(m, "place_order_cstr")
    .def(py::init<>())
    .def("name", &PlaceOrderCstr::name, py::return_value_policy::reference_internal)
    .def("idx", &PlaceOrderCstr::idx)
    .def("cell", py::overload_cast<const Int, const Int>(&PlaceOrderCstr::cell), py::return_value_policy::reference_internal)
    .def("cell_const", py::overload_cast<const Int, const Int>(&PlaceOrderCstr::cell, py::const_), py::return_value_policy::reference_internal)
    .def("num_cells", &PlaceOrderCstr::numCells)
    .def("num_cell_groups", &PlaceOrderCstr::numCellGroups)
    .def("dir", &PlaceOrderCstr::dir)
    .def("is_strict", &PlaceOrderCstr::isStrict)
    .def("is_weak", &PlaceOrderCstr::isWeak);
}

void initPlaceAlignCstrAPI(py::module& m)
{
  py::class_<PlaceAlignCstr>(m, "place_align_cstr")
    .def(py::init<>())
    .def("name", &PlaceAlignCstr::name, py::return_value_policy::reference_internal)
    .def("idx", &PlaceAlignCstr::idx)
    .def("cell", py::overload_cast<const Int>(&PlaceAlignCstr::cell), py::return_value_policy::reference_internal)
    .def("cell_const", py::overload_cast<const Int>(&PlaceAlignCstr::cell, py::const_), py::return_value_policy::reference_internal)
    .def("num_cells", &PlaceAlignCstr::numCells)
    .def("region", py::overload_cast<const Int>(&PlaceAlignCstr::region), py::return_value_policy::reference_internal)
    .def("region_const", py::overload_cast<const Int>(&PlaceAlignCstr::region, py::const_), py::return_value_policy::reference_internal)
    .def("num_regions", &PlaceAlignCstr::numRegions)
    .def("place_array_cstr", py::overload_cast<const Int>(&PlaceAlignCstr::placeArrayCstr), py::return_value_policy::reference_internal)
    .def("place_array_cstr_const", py::overload_cast<const Int>(&PlaceAlignCstr::placeArrayCstr, py::const_), py::return_value_policy::reference_internal)
    .def("num_place_array_cstrs", &PlaceAlignCstr::numPlaceArrayCstrs)
    .def("is_hor", &PlaceAlignCstr::isHor)
    .def("is_ver", &PlaceAlignCstr::isVer)
    .def("is_high", &PlaceAlignCstr::isHigh)
    .def("is_low", &PlaceAlignCstr::isLow);

}

void initPlaceDisjointCstrAPI(py::module& m)
{
  py::class_<PlaceDisjointCstr>(m, "place_disjoint_cstr")
    .def(py::init<>())
    .def("name", &PlaceDisjointCstr::name, py::return_value_policy::reference_internal)
    .def("idx", &PlaceDisjointCstr::idx, py::return_value_policy::reference_internal)
    .def("cell", py::overload_cast<const Int>(&PlaceDisjointCstr::cell), py::return_value_policy::reference_internal)
    .def("cell_const", py::overload_cast<const Int>(&PlaceDisjointCstr::cell, py::const_), py::return_value_policy::reference_internal)
    .def("num_cells", &PlaceDisjointCstr::numCells)
    .def("is_row", &PlaceDisjointCstr::isRow)
    .def("is_col", &PlaceDisjointCstr::isCol);

}

void initPlaceRowCstrAPI(py::module& m)
{
  py::class_<PlaceRowCstr>(m, "place_row_cstr")
    .def(py::init<>())
    .def("name", &PlaceRowCstr::name, py::return_value_policy::reference_internal)
    .def("idx", &PlaceRowCstr::idx)
    .def("cell", py::overload_cast<const Int>(&PlaceRowCstr::cell), py::return_value_policy::reference_internal)
    .def("cell_const", py::overload_cast<const Int>(&PlaceRowCstr::cell, py::const_), py::return_value_policy::reference_internal)
    .def("num_cells", &PlaceRowCstr::numCells)
    .def("is_odd", &PlaceRowCstr::isOdd)
    .def("is_even", &PlaceRowCstr::isEven);
}

void initRouteSymCstrAPI(py::module& m)
{
  py::class_<RouteSymCstr>(m, "route_sym_cstr")
    .def(py::init<>())
    .def("name", &RouteSymCstr::name, py::return_value_policy::reference_internal)
    .def("idx", &RouteSymCstr::idx)
    .def("axis", &RouteSymCstr::axis)
    .def("net", py::overload_cast<const Int>(&RouteSymCstr::net), py::return_value_policy::reference_internal)
    .def("net_const", py::overload_cast<const Int>(&RouteSymCstr::net, py::const_), py::return_value_policy::reference_internal)
    .def("num_nets", &RouteSymCstr::numNets)
    .def("sym_part", py::overload_cast<const Int>(&RouteSymCstr::symPart, py::const_))
    .def("sym_part", py::overload_cast<const Net&>(&RouteSymCstr::symPart, py::const_))
    .def("is_part_a", py::overload_cast<const Int>(&RouteSymCstr::isPartA, py::const_))
    .def("is_part_a", py::overload_cast<const Net&>(&RouteSymCstr::isPartA, py::const_))
    .def("is_part_b", py::overload_cast<const Int>(&RouteSymCstr::isPartB, py::const_))
    .def("is_part_b", py::overload_cast<const Net&>(&RouteSymCstr::isPartB, py::const_))
    .def("is_self_sym", py::overload_cast<const Int>(&RouteSymCstr::isSelfSym, py::const_))
    .def("is_self_sym", py::overload_cast<const Net&>(&RouteSymCstr::isSelfSym, py::const_))
    .def("num_self_syms", &RouteSymCstr::numSelfSyms)
    .def("num_part_as", &RouteSymCstr::numPartAs)
    .def("has_self_sym", &RouteSymCstr::hasSelfSym)
    .def("net_idx_to_idx", &RouteSymCstr::netIdx2Idx)
    .def("net_idx_to_sym_idx", &RouteSymCstr::netIdx2SymIdx)
    .def("sym_net", py::overload_cast<const Int>(&RouteSymCstr::symNet), py::return_value_policy::reference_internal)
    .def("sym_net", py::overload_cast<const Net&>(&RouteSymCstr::symNet), py::return_value_policy::reference_internal)
    .def("sym_net_const", py::overload_cast<const Int>(&RouteSymCstr::symNet, py::const_), py::return_value_policy::reference_internal)
    .def("sym_net_const", py::overload_cast<const Net&>(&RouteSymCstr::symNet, py::const_), py::return_value_policy::reference_internal)
    .def("region", &RouteSymCstr::region, py::return_value_policy::reference_internal);

}

void initRoutePathMatchCstrAPI(py::module& m)
{
  py::class_<RoutePathMatchCstr::PathCstr>(m, "path_cstr")
    .def(py::init<>())
    .def("idx", &RoutePathMatchCstr::PathCstr::idx)
    .def("src_pin", py::overload_cast<const Int>(&RoutePathMatchCstr::PathCstr::srcPin), py::return_value_policy::reference_internal)
    .def("src_pin_const", py::overload_cast<const Int>(&RoutePathMatchCstr::PathCstr::srcPin, py::const_), py::return_value_policy::reference_internal)
    .def("tar_pin", py::overload_cast<const Int>(&RoutePathMatchCstr::PathCstr::tarPin), py::return_value_policy::reference_internal)
    .def("tar_pin_const", py::overload_cast<const Int>(&RoutePathMatchCstr::PathCstr::tarPin, py::const_), py::return_value_policy::reference_internal)
    .def("conn", py::overload_cast<const Int>(&RoutePathMatchCstr::PathCstr::conn), py::return_value_policy::reference_internal)
    .def("conn_const", py::overload_cast<const Int>(&RoutePathMatchCstr::PathCstr::conn, py::const_), py::return_value_policy::reference_internal)
    .def("v_conns", py::overload_cast<>(&RoutePathMatchCstr::PathCstr::vConns, py::const_), py::return_value_policy::reference_internal)
    .def("has_net", &RoutePathMatchCstr::PathCstr::hasNet)
    .def("num_conns", &RoutePathMatchCstr::PathCstr::numConns);

  py::class_<RoutePathMatchCstr>(m, "route_path_match_cstr")
    .def(py::init<>())
    .def("name", &RoutePathMatchCstr::name, py::return_value_policy::reference_internal)
    .def("idx", &RoutePathMatchCstr::idx)
    .def("net", py::overload_cast<const Int>(&RoutePathMatchCstr::net), py::return_value_policy::reference_internal)
    .def("net_const", py::overload_cast<const Int>(&RoutePathMatchCstr::net, py::const_), py::return_value_policy::reference_internal)
    .def("v_nets", py::overload_cast<>(&RoutePathMatchCstr::vpNets), py::return_value_policy::reference_internal)
    .def("num_nets", &RoutePathMatchCstr::numNets)
    .def("path_cstr", py::overload_cast<const Int>(&RoutePathMatchCstr::pathCstr), py::return_value_policy::reference_internal)
    .def("path_cstr_const", py::overload_cast<const Int>(&RoutePathMatchCstr::pathCstr, py::const_), py::return_value_policy::reference_internal)
    .def("path_cstr", py::overload_cast<const Net*>(&RoutePathMatchCstr::pathCstr), py::return_value_policy::reference_internal)
    .def("path_cstr_const", py::overload_cast<const Net*>(&RoutePathMatchCstr::pathCstr, py::const_), py::return_value_policy::reference_internal)
    .def("v_path_cstrs", py::overload_cast<>(&RoutePathMatchCstr::vPathCstrs), py::return_value_policy::reference_internal)
    .def("num_path_cstrs", &RoutePathMatchCstr::numPathCstrs)
    .def("net_idx", &RoutePathMatchCstr::netIdx)
    .def("has_net", &RoutePathMatchCstr::hasNet);

}

void initNetPrioCstrAPI(py::module& m)
{
  py::class_<NetPrioCstr>(m, "net_prio_cstr")
    .def(py::init<>())
    .def("name", &NetPrioCstr::name, py::return_value_policy::reference_internal)
    .def("idx", &NetPrioCstr::idx)
    .def("net", py::overload_cast<const Int>(&NetPrioCstr::net), py::return_value_policy::reference_internal)
    .def("net_const", py::overload_cast<const Int>(&NetPrioCstr::net, py::const_), py::return_value_policy::reference_internal)
    .def("num_nets", &NetPrioCstr::numNets)
    .def("priority", &NetPrioCstr::priority);
}

void initCirAPI(py::module& m)
{
  py::class_<CirDB>(m, "cir_db")
    .def(py::init<>())
    .def("name", &CirDB::name, py::return_value_policy::reference_internal)
    .def("phys_res", &CirDB::physRes)

    .def("bbox", py::overload_cast<>(&CirDB::bbox), py::return_value_policy::reference_internal)
    .def("bbox_const", py::overload_cast<>(&CirDB::bbox, py::const_), py::return_value_policy::reference_internal)
    .def("routing_bbox", py::overload_cast<>(&CirDB::routingBbox), py::return_value_policy::reference_internal)
    .def("routing_bbox_const", py::overload_cast<>(&CirDB::routingBbox, py::const_), py::return_value_policy::reference_internal)

    .def("default_place_grid_const", &CirDB::defaultPlaceGrid, py::return_value_policy::reference_internal)
    .def("place_grid_const", py::overload_cast<const Int>(&CirDB::placeGrid, py::const_), py::return_value_policy::reference_internal)
    .def("place_grid_const", py::overload_cast<const String&>(&CirDB::placeGrid, py::const_), py::return_value_policy::reference_internal)
    .def("name_to_place_grid_idx", &CirDB::name2PlaceGridIdx)
    .def("num_place_grids", &CirDB::numPlaceGrids)
    .def("has_place_grid", &CirDB::hasPlaceGrid)
    .def("v_place_grids", &CirDB::vpPlaceGrids, py::return_value_policy::reference_internal)

    .def("default_route_grid_const", &CirDB::defaultRouteGrid, py::return_value_policy::reference_internal)
    .def("route_grid_const", py::overload_cast<const Int>(&CirDB::routeGrid, py::const_), py::return_value_policy::reference_internal)
    .def("route_grid_const", py::overload_cast<const String&>(&CirDB::routeGrid, py::const_), py::return_value_policy::reference_internal)
    .def("name_to_route_grid_idx", &CirDB::name2RouteGridIdx)
    .def("num_default_route_grids", &CirDB::numDefaultRouteGrids)
    .def("num_route_grids", &CirDB::numRouteGrids)
    .def("has_route_grid", &CirDB::hasRouteGrid)
    .def("v_route_grids", &CirDB::vpRouteGrids, py::return_value_policy::reference_internal)

    .def("default_power_route_grid", &CirDB::defaultPowerRouteGrid, py::return_value_policy::reference_internal)
    .def("power_route_grid", py::overload_cast<const Int>(&CirDB::powerRouteGrid, py::const_), py::return_value_policy::reference_internal)
    .def("power_route_grid", py::overload_cast<const String&>(&CirDB::powerRouteGrid, py::const_), py::return_value_policy::reference_internal)
    .def("num_default_power_route_grids", &CirDB::numDefaultPowerRouteGrids)
    .def("num_power_route_grids", &CirDB::numPowerRouteGrids)
    .def("has_power_route_grid", &CirDB::hasPowerRouteGrid)
    .def("v_power_route_grids", &CirDB::vpPowerRouteGrids, py::return_value_policy::reference_internal)

    .def("layer_const", py::overload_cast<const Int>(&CirDB::layer, py::const_), py::return_value_policy::reference_internal)
    .def("layer_const", py::overload_cast<const String&>(&CirDB::layer, py::const_), py::return_value_policy::reference_internal)
    .def("name_to_layer_idx", &CirDB::name2LayerIdx)
    .def("num_layers", &CirDB::numLayers)
    .def("has_layer", &CirDB::hasLayer)
    .def("v_layers", &CirDB::vpLayers, py::return_value_policy::reference_internal)

    .def("metal_layer_metal", py::overload_cast<const Int>(&CirDB::metalLayer, py::const_), py::return_value_policy::reference_internal)
    .def("metal_layer_metal", py::overload_cast<const String&>(&CirDB::metalLayer, py::const_), py::return_value_policy::reference_internal)
    .def("name_to_metal_layer_idx", &CirDB::name2MetalLayerIdx)
    .def("num_metal_layers", &CirDB::numMetalLayers)
    .def("has_metal_layer", &CirDB::hasMetalLayer)
    .def("v_metal_layers", &CirDB::vpMetalLayers, py::return_value_policy::reference_internal)

    .def("cut_layer_const", py::overload_cast<const Int>(&CirDB::cutLayer, py::const_), py::return_value_policy::reference_internal)
    .def("cut_layer_const", py::overload_cast<const String&>(&CirDB::cutLayer, py::const_), py::return_value_policy::reference_internal)
    .def("name_to_cut_layer_idx", &CirDB::name2CutLayerIdx)
    .def("num_cut_layers", &CirDB::numCutLayers)
    .def("has_cut_layer", &CirDB::hasCutLayer)
    .def("v_cut_layers", &CirDB::vpCutLayers, py::return_value_policy::reference_internal)

    .def("prim_const", py::overload_cast<const Int>(&CirDB::prim, py::const_), py::return_value_policy::reference_internal)
    .def("prim_const", py::overload_cast<const String&>(&CirDB::prim, py::const_), py::return_value_policy::reference_internal)
    .def("name_to_prim_idx", &CirDB::name2PrimIdx)
    .def("num_prims", &CirDB::numPrims)
    .def("has_prim", &CirDB::hasPrim)
    .def("v_prims", &CirDB::vpPrims, py::return_value_policy::reference_internal)

    .def("dev_map_const", py::overload_cast<const Int>(&CirDB::devMap, py::const_), py::return_value_policy::reference_internal)
    .def("dev_map_const", py::overload_cast<const String&>(&CirDB::devMap, py::const_), py::return_value_policy::reference_internal)
    .def("num_dev_maps", &CirDB::numDevMaps)
    .def("has_dev_map", &CirDB::hasDevMap)
    .def("v_dev_maps", &CirDB::vpDevMaps, py::return_value_policy::reference_internal)

    .def("cell", py::overload_cast<const Int>(&CirDB::cell), py::return_value_policy::reference_internal)
    .def("cell", py::overload_cast<const String&>(&CirDB::cell), py::return_value_policy::reference_internal)
    .def("cell_const", py::overload_cast<const Int>(&CirDB::cell, py::const_), py::return_value_policy::reference_internal)
    .def("cell_const", py::overload_cast<const String&>(&CirDB::cell, py::const_), py::return_value_policy::reference_internal)
    .def("name_to_cell_idx", &CirDB::name2CellIdx)
    .def("has_cell", &CirDB::hasCell)
    .def("num_cells", &CirDB::numCells)
    .def("v_cells", py::overload_cast<>(&CirDB::vpCells), py::return_value_policy::reference_internal)

    .def("v_submodule_cell_ids", &CirDB::vSubmoduleCellIds, py::return_value_policy::reference_internal)
    .def("num_submodules", &CirDB::numSubmodules)
    .def("has_submodule", &CirDB::hasSubmodule)
    .def("v_base_cell_ids", &CirDB::vBaseCellIds, py::return_value_policy::reference_internal)
    .def("num_base_cells", &CirDB::numBaseCells)
    .def("has_base_cell", &CirDB::hasBaseCell)

    .def("pin", py::overload_cast<const Int>(&CirDB::pin), py::return_value_policy::reference_internal)
    .def("pin", py::overload_cast<const String&>(&CirDB::pin), py::return_value_policy::reference_internal)
    .def("pin_const", py::overload_cast<const Int>(&CirDB::pin, py::const_), py::return_value_policy::reference_internal)
    .def("pin_const", py::overload_cast<const String&>(&CirDB::pin, py::const_), py::return_value_policy::reference_internal)
    .def("name_to_pin_idx", &CirDB::name2PinIdx)
    .def("num_pins", py::overload_cast<>(&CirDB::numPins, py::const_))
    .def("has_pin", &CirDB::hasPin)
    .def("v_pins", py::overload_cast<>(&CirDB::vpPins), py::return_value_policy::reference_internal)

    .def("edge_prim_const", &CirDB::edgePrim, py::return_value_policy::reference_internal)
    .def("num_edge_prims", &CirDB::numEdgePrims)
    .def("dummy_prim_const", &CirDB::dummyPrim, py::return_value_policy::reference_internal)
    .def("num_dummy_prims", &CirDB::numDummyPrims)

    .def("via", py::overload_cast<const Int>(&CirDB::via), py::return_value_policy::reference_internal)
    .def("via", py::overload_cast<const String&>(&CirDB::via), py::return_value_policy::reference_internal)
    .def("via_const", py::overload_cast<const Int>(&CirDB::via, py::const_), py::return_value_policy::reference_internal)
    .def("via_const", py::overload_cast<const String&>(&CirDB::via, py::const_), py::return_value_policy::reference_internal)
    .def("name_to_via_idx", &CirDB::name2ViaIdx)
    .def("num_vias", &CirDB::numVias)
    .def("has_via", &CirDB::hasVia)
    .def("v_vias", &CirDB::vpVias, py::return_value_policy::reference_internal)

    .def("net", py::overload_cast<const Int>(&CirDB::net), py::return_value_policy::reference_internal)
    .def("net", py::overload_cast<const String&>(&CirDB::net), py::return_value_policy::reference_internal)
    .def("net_const", py::overload_cast<const Int>(&CirDB::net, py::const_), py::return_value_policy::reference_internal)
    .def("net_const", py::overload_cast<const String&>(&CirDB::net, py::const_), py::return_value_policy::reference_internal)
    .def("name_to_net_idx", &CirDB::name2NetIdx)
    .def("num_nets", &CirDB::numNets)
    .def("has_net", &CirDB::hasNet)
    .def("v_nets", &CirDB::vpNets, py::return_value_policy::reference_internal)

    .def("io_pin", py::overload_cast<const Int>(&CirDB::ioPin), py::return_value_policy::reference_internal)
    .def("io_pin", py::overload_cast<const String&>(&CirDB::ioPin), py::return_value_policy::reference_internal)
    .def("io_pin_const", py::overload_cast<const Int>(&CirDB::ioPin, py::const_), py::return_value_policy::reference_internal)
    .def("io_pin_const", py::overload_cast<const String&>(&CirDB::ioPin, py::const_), py::return_value_policy::reference_internal)
    .def("name_to_io_pin_idx", &CirDB::name2IOPinIdx)
    .def("num_io_pins", &CirDB::numIOPins)
    .def("has_io_pin", &CirDB::hasIOPin)
    .def("v_io_pins", &CirDB::vpIOPins, py::return_value_policy::reference_internal)
    .def("io_pin_loc", &CirDB::ioPinLoc)
    .def("io_pin_bbox", py::overload_cast<const Int>(&CirDB::ioPinBbox), py::return_value_policy::reference_internal)
    .def("io_pin_bbox_const", py::overload_cast<const Int>(&CirDB::ioPinBbox, py::const_), py::return_value_policy::reference_internal)
    .def("io_pin_l", py::overload_cast<const Int>(&CirDB::ioPinL), py::return_value_policy::reference_internal)
    .def("io_pin_l_const", py::overload_cast<const Int>(&CirDB::ioPinL, py::const_), py::return_value_policy::reference_internal)
    .def("io_pin_r", py::overload_cast<const Int>(&CirDB::ioPinR), py::return_value_policy::reference_internal)
    .def("io_pin_r_const", py::overload_cast<const Int>(&CirDB::ioPinR, py::const_), py::return_value_policy::reference_internal)
    .def("io_pin_b", py::overload_cast<const Int>(&CirDB::ioPinB), py::return_value_policy::reference_internal)
    .def("io_pin_b_const", py::overload_cast<const Int>(&CirDB::ioPinB, py::const_), py::return_value_policy::reference_internal)
    .def("io_pin_t", py::overload_cast<const Int>(&CirDB::ioPinT), py::return_value_policy::reference_internal)
    .def("io_pin_t_const", py::overload_cast<const Int>(&CirDB::ioPinT, py::const_), py::return_value_policy::reference_internal)
    .def("num_left_io_pins", &CirDB::numLeftIOPins)
    .def("num_right_io_pins", &CirDB::numRightIOPins)
    .def("num_bottom_io_pins", &CirDB::numBottomIOPins)
    .def("num_top_io_pins", &CirDB::numTopIOPins)

    .def("region", py::overload_cast<const Int>(&CirDB::region), py::return_value_policy::reference_internal)
    .def("region", py::overload_cast<const String&>(&CirDB::region), py::return_value_policy::reference_internal)
    .def("region_const", py::overload_cast<const Int>(&CirDB::region, py::const_), py::return_value_policy::reference_internal)
    .def("region_const", py::overload_cast<const String&>(&CirDB::region, py::const_), py::return_value_policy::reference_internal)
    .def("name_to_region_idx", &CirDB::name2RegionIdx)
    .def("num_regions", &CirDB::numRegions)
    .def("has_region", &CirDB::hasRegion)
    .def("v_regions", &CirDB::vpRegions, py::return_value_policy::reference_internal)

    .def("pre_place_cstr", py::overload_cast<const Int>(&CirDB::prePlaceCstr), py::return_value_policy::reference_internal)
    .def("pre_place_cstr", py::overload_cast<const String&>(&CirDB::prePlaceCstr), py::return_value_policy::reference_internal)
    .def("pre_place_cstr_const", py::overload_cast<const Int>(&CirDB::prePlaceCstr, py::const_), py::return_value_policy::reference_internal)
    .def("pre_place_cstr_const", py::overload_cast<const String&>(&CirDB::prePlaceCstr, py::const_), py::return_value_policy::reference_internal)
    .def("name_to_pre_place_cstr_idx", &CirDB::name2PrePlaceCstrIdx)
    .def("num_pre_place_cstrs", &CirDB::numPrePlaceCstrs)
    .def("has_pre_place_cstr", &CirDB::hasPrePlaceCstr)
    .def("v_pre_place_cstrs", py::overload_cast<>(&CirDB::vpPrePlaceCstrs, py::const_), py::return_value_policy::reference_internal)

    .def("place_sym_cstr", py::overload_cast<const Int>(&CirDB::placeSymCstr), py::return_value_policy::reference_internal)
    .def("place_sym_cstr", py::overload_cast<const String&>(&CirDB::placeSymCstr), py::return_value_policy::reference_internal)
    .def("place_sym_cstr_const", py::overload_cast<const Int>(&CirDB::placeSymCstr, py::const_), py::return_value_policy::reference_internal)
    .def("place_sym_cstr_const", py::overload_cast<const String&>(&CirDB::placeSymCstr, py::const_), py::return_value_policy::reference_internal)
    .def("name_to_place_sym_cstr_idx", &CirDB::name2PlaceSymCstrIdx)
    .def("num_place_sym_cstrs", &CirDB::numPlaceSymCstrs)
    .def("has_place_sym_cstr", &CirDB::hasPlaceSymCstr)
    .def("v_place_sym_cstrs", py::overload_cast<>(&CirDB::vpPlaceSymCstrs, py::const_), py::return_value_policy::reference_internal)

    .def("place_array_cstr", py::overload_cast<const Int>(&CirDB::placeArrayCstr), py::return_value_policy::reference_internal)
    .def("place_array_cstr", py::overload_cast<const String&>(&CirDB::placeArrayCstr), py::return_value_policy::reference_internal)
    .def("place_array_cstr_const", py::overload_cast<const Int>(&CirDB::placeArrayCstr, py::const_), py::return_value_policy::reference_internal)
    .def("place_array_cstr_const", py::overload_cast<const String&>(&CirDB::placeArrayCstr, py::const_), py::return_value_policy::reference_internal)
    .def("name_to_place_array_cstr_idx", &CirDB::name2PlaceArrayCstrIdx)
    .def("num_place_array_cstrs", &CirDB::numPlaceArrayCstrs)
    .def("has_place_array_cstr", &CirDB::hasPlaceArrayCstr)
    .def("v_place_array_cstrs", py::overload_cast<>(&CirDB::vpPlaceArrayCstrs, py::const_), py::return_value_policy::reference_internal)

    .def("place_cluster_cstr", py::overload_cast<const Int>(&CirDB::placeClusterCstr), py::return_value_policy::reference_internal)
    .def("place_cluster_cstr", py::overload_cast<const String&>(&CirDB::placeClusterCstr), py::return_value_policy::reference_internal)
    .def("place_cluster_cstr_const", py::overload_cast<const Int>(&CirDB::placeClusterCstr, py::const_), py::return_value_policy::reference_internal)
    .def("place_cluster_cstr_const", py::overload_cast<const String&>(&CirDB::placeClusterCstr, py::const_), py::return_value_policy::reference_internal)
    .def("name_to_place_cluster_cstr_idx", &CirDB::name2PlaceClusterCstrIdx)
    .def("num_place_cluster_cstrs", &CirDB::numPlaceClusterCstrs)
    .def("has_place_cluster_cstr", &CirDB::hasPlaceClusterCstr)
    .def("v_place_cluster_cstrs", py::overload_cast<>(&CirDB::vpPlaceClusterCstrs, py::const_), py::return_value_policy::reference_internal)

    .def("place_ext_cstr", py::overload_cast<const Int>(&CirDB::placeExtCstr), py::return_value_policy::reference_internal)
    .def("place_ext_cstr", py::overload_cast<const String&>(&CirDB::placeExtCstr), py::return_value_policy::reference_internal)
    .def("place_ext_cstr_const", py::overload_cast<const Int>(&CirDB::placeExtCstr, py::const_), py::return_value_policy::reference_internal)
    .def("place_ext_cstr_const", py::overload_cast<const String&>(&CirDB::placeExtCstr, py::const_), py::return_value_policy::reference_internal)
    .def("name_to_place_ext_cstr_idx", &CirDB::name2PlaceExtCstrIdx)
    .def("num_place_ext_cstrs", &CirDB::numPlaceExtCstrs)
    .def("has_place_ext_cstr", &CirDB::hasPlaceExtCstr)
    .def("v_place_ext_cstrs", py::overload_cast<>(&CirDB::vpPlaceExtCstrs, py::const_), py::return_value_policy::reference_internal)

    .def("place_edge_dist_cstr", py::overload_cast<const Int>(&CirDB::placeEdgeDistCstr), py::return_value_policy::reference_internal)
    .def("place_edge_dist_cstr", py::overload_cast<const String&>(&CirDB::placeEdgeDistCstr), py::return_value_policy::reference_internal)
    .def("place_edge_dist_cstr_const", py::overload_cast<const Int>(&CirDB::placeEdgeDistCstr, py::const_), py::return_value_policy::reference_internal)
    .def("place_edge_dist_cstr_const", py::overload_cast<const String&>(&CirDB::placeEdgeDistCstr, py::const_), py::return_value_policy::reference_internal)
    .def("name_to_place_edge_dist_cstr_idx", &CirDB::name2PlaceEdgeDistCstrIdx)
    .def("num_place_edge_dist_cstrs", &CirDB::numPlaceEdgeDistCstrs)
    .def("has_place_edge_dist_cstr", &CirDB::hasPlaceEdgeDistCstr)
    .def("v_place_edge_dist_cstrs", py::overload_cast<>(&CirDB::vpPlaceEdgeDistCstrs, py::const_), py::return_value_policy::reference_internal)

    .def("place_order_cstr", py::overload_cast<const Int>(&CirDB::placeOrderCstr), py::return_value_policy::reference_internal)
    .def("place_order_cstr", py::overload_cast<const String&>(&CirDB::placeOrderCstr), py::return_value_policy::reference_internal)
    .def("place_order_cstr_const", py::overload_cast<const Int>(&CirDB::placeOrderCstr, py::const_), py::return_value_policy::reference_internal)
    .def("place_order_cstr_const", py::overload_cast<const String&>(&CirDB::placeOrderCstr, py::const_), py::return_value_policy::reference_internal)
    .def("name_to_place_order_cstr_idx", &CirDB::name2PlaceOrderCstrIdx)
    .def("num_place_order_cstrs", &CirDB::numPlaceOrderCstrs)
    .def("has_place_order_cstr", &CirDB::hasPlaceOrderCstr)
    .def("v_place_order_cstrs", py::overload_cast<>(&CirDB::vpPlaceOrderCstrs, py::const_), py::return_value_policy::reference_internal)

    .def("place_align_cstr", py::overload_cast<const Int>(&CirDB::placeAlignCstr), py::return_value_policy::reference_internal)
    .def("place_align_cstr", py::overload_cast<const String&>(&CirDB::placeAlignCstr), py::return_value_policy::reference_internal)
    .def("place_align_cstr_const", py::overload_cast<const Int>(&CirDB::placeAlignCstr, py::const_), py::return_value_policy::reference_internal)
    .def("place_align_cstr_const", py::overload_cast<const String&>(&CirDB::placeAlignCstr, py::const_), py::return_value_policy::reference_internal)
    .def("name_to_place_align_cstr_idx", &CirDB::name2PlaceAlignCstrIdx)
    .def("num_place_align_cstrs", &CirDB::numPlaceAlignCstrs)
    .def("has_place_align_cstr", &CirDB::hasPlaceAlignCstr)
    .def("v_place_align_cstrs", py::overload_cast<>(&CirDB::vpPlaceAlignCstrs, py::const_), py::return_value_policy::reference_internal)

    .def("place_disjoint_cstr", py::overload_cast<const Int>(&CirDB::placeDisjointCstr), py::return_value_policy::reference_internal)
    .def("place_disjoint_cstr", py::overload_cast<const String&>(&CirDB::placeDisjointCstr), py::return_value_policy::reference_internal)
    .def("place_disjoint_cstr_const", py::overload_cast<const Int>(&CirDB::placeDisjointCstr, py::const_), py::return_value_policy::reference_internal)
    .def("place_disjoint_cstr_const", py::overload_cast<const String&>(&CirDB::placeDisjointCstr, py::const_), py::return_value_policy::reference_internal)
    .def("name_to_place_disjoint_cstr_idx", &CirDB::name2PlaceDisjointCstrIdx)
    .def("num_place_disjoint_cstrs", &CirDB::numPlaceDisjointCstrs)
    .def("has_place_disjoint_cstr", &CirDB::hasPlaceDisjointCstr)
    .def("v_place_disjoint_cstrs", py::overload_cast<>(&CirDB::vpPlaceDisjointCstrs, py::const_), py::return_value_policy::reference_internal)
    
    .def("place_row_cstr", py::overload_cast<const Int>(&CirDB::placeRowCstr), py::return_value_policy::reference_internal)
    .def("place_row_cstr", py::overload_cast<const String&>(&CirDB::placeRowCstr), py::return_value_policy::reference_internal)
    .def("place_row_cstr_const", py::overload_cast<const Int>(&CirDB::placeRowCstr, py::const_), py::return_value_policy::reference_internal)
    .def("place_row_cstr_const", py::overload_cast<const String&>(&CirDB::placeRowCstr, py::const_), py::return_value_policy::reference_internal)
    .def("name_to_place_row_cstr_idx", &CirDB::name2PlaceRowCstrIdx)
    .def("num_place_row_cstrs", &CirDB::numPlaceRowCstrs)
    .def("has_place_row_cstr", &CirDB::hasPlaceRowCstr)
    .def("v_place_row_cstrs", py::overload_cast<>(&CirDB::vpPlaceRowCstrs, py::const_), py::return_value_policy::reference_internal)

    .def("route_sym_cstr", py::overload_cast<const Int>(&CirDB::routeSymCstr), py::return_value_policy::reference_internal)
    .def("route_sym_cstr", py::overload_cast<const String&>(&CirDB::routeSymCstr), py::return_value_policy::reference_internal)
    .def("route_sym_cstr_const", py::overload_cast<const Int>(&CirDB::routeSymCstr, py::const_), py::return_value_policy::reference_internal)
    .def("route_sym_cstr_const", py::overload_cast<const String&>(&CirDB::routeSymCstr, py::const_), py::return_value_policy::reference_internal)
    .def("name_to_route_sym_cstr_idx", &CirDB::name2RouteSymCstrIdx)
    .def("num_route_sym_cstrs", &CirDB::numRouteSymCstrs)
    .def("has_route_sym_cstr", &CirDB::hasRouteSymCstr)
    .def("v_route_sym_cstrs", py::overload_cast<>(&CirDB::vpRouteSymCstrs, py::const_), py::return_value_policy::reference_internal)

    .def("route_path_match_cstr", py::overload_cast<const Int>(&CirDB::routePathMatchCstr), py::return_value_policy::reference_internal)
    .def("route_path_match_cstr", py::overload_cast<const String&>(&CirDB::routePathMatchCstr), py::return_value_policy::reference_internal)
    .def("route_path_match_cstr_const", py::overload_cast<const Int>(&CirDB::routePathMatchCstr, py::const_), py::return_value_policy::reference_internal)
    .def("route_path_match_cstr_const", py::overload_cast<const String&>(&CirDB::routePathMatchCstr, py::const_), py::return_value_policy::reference_internal)
    .def("name_to_route_path_match_cstr_idx", &CirDB::name2RoutePathMatchCstrIdx)
    .def("num_route_path_match_cstrs", &CirDB::numRoutePathMatchCstrs)
    .def("has_route_path_match_cstr", &CirDB::hasRoutePathMatchCstr)
    .def("v_route_path_match_cstrs", py::overload_cast<>(&CirDB::vpRoutePathMatchCstrs, py::const_), py::return_value_policy::reference_internal)

    .def("net_prio_cstr", py::overload_cast<const Int>(&CirDB::netPrioCstr), py::return_value_policy::reference_internal)
    .def("net_prio_cstr", py::overload_cast<const String&>(&CirDB::netPrioCstr), py::return_value_policy::reference_internal)
    .def("net_prio_cstr_const", py::overload_cast<const Int>(&CirDB::netPrioCstr, py::const_), py::return_value_policy::reference_internal)
    .def("net_prio_cstr_const", py::overload_cast<const String&>(&CirDB::netPrioCstr, py::const_), py::return_value_policy::reference_internal)
    .def("name_to_net_prio_cstr_idx", &CirDB::name2NetPrioCstrIdx)
    .def("num_net_prio_cstrs", &CirDB::numNetPrioCstrs)
    .def("has_net_prio_cstr", &CirDB::hasNetPrioCstr)
    .def("v_net_prio_cstrs", py::overload_cast<>(&CirDB::vpNetPrioCstrs, py::const_), py::return_value_policy::reference_internal)

    .def("pin", py::overload_cast<const Int, const Int>(&CirDB::pin), py::return_value_policy::reference_internal)
    .def("pin_const", py::overload_cast<const Int, const Int>(&CirDB::pin, py::const_), py::return_value_policy::reference_internal)
    .def("num_pins", py::overload_cast<const Int>(&CirDB::numPins, py::const_))
    .def("v_pins", py::overload_cast<const Int>(&CirDB::vpPins, py::const_), py::return_value_policy::reference_internal)

    .def("obs", py::overload_cast<const Int, const Int>(&CirDB::obs), py::return_value_policy::reference_internal)
    .def("obs", py::overload_cast<const Int, const Int>(&CirDB::obs, py::const_), py::return_value_policy::reference_internal)
    .def("num_obss", py::overload_cast<const Int>(&CirDB::numObs, py::const_))
    .def("v_obs", py::overload_cast<const Int>(&CirDB::vpObs, py::const_), py::return_value_policy::reference_internal)

    .def("wsp_bbox", py::overload_cast<const Int>(&CirDB::wspBbox), py::return_value_policy::reference_internal)
    .def("wsp_bbox_const", py::overload_cast<const Int>(&CirDB::wspBbox, py::const_), py::return_value_policy::reference_internal)
    .def("v_wsp_bboxes", py::overload_cast<>(&CirDB::vWspBboxes), py::return_value_policy::reference_internal)
    .def("v_wsp_bboxes_const", py::overload_cast<>(&CirDB::vWspBboxes, py::const_), py::return_value_policy::reference_internal)
    .def("wsp_fill_drw", py::overload_cast<const Int>(&CirDB::wspFillDrw), py::return_value_policy::reference_internal)
    .def("wsp_fill_drw_const", py::overload_cast<const Int>(&CirDB::wspFillDrw, py::const_), py::return_value_policy::reference_internal)
    .def("v_wsp_fill_drws", py::overload_cast<>(&CirDB::vWspFillDrws), py::return_value_policy::reference_internal)
    .def("v_wsp_fill_drws_const", py::overload_cast<>(&CirDB::vWspFillDrws, py::const_), py::return_value_policy::reference_internal)
    .def("update_bbox", &CirDB::updateBbox)
    .def("group_cells", &CirDB::groupCells)
    .def("init_spatials", &CirDB::initSpatials)
    .def("build_spatial_pins_and_obs", &CirDB::buildSpatialPinsNObs)
    .def("build_spatial_net", &CirDB::buildSpatialNet)
    .def("build_spatial_nets", &CirDB::buildSpatialNets)
    .def("set_io_pin_bbox", &CirDB::setIOPinBbox)
    .def("gen_routing_bbox", &CirDB::genRoutingBbox)
    .def("gen_wsp", &CirDB::genWsp)

    .def("seg_res", &CirDB::segRes)
    .def("path_length", &CirDB::pathLength)
    .def("path_vias", &CirDB::pathVias)
    .def("path_length_vias", &CirDB::pathLengthVias)
    .def("path_res", py::overload_cast<const Vector<Segment3d<Int>>&>(&CirDB::pathRes, py::const_))
    .def("path_res", py::overload_cast<const Vector<Vector<Segment3d<Int>>>&>(&CirDB::pathRes, py::const_))
    .def("path_stats", py::overload_cast<const Vector<Segment3d<Int>>&>(&CirDB::pathStats, py::const_))
    .def("path_stats", py::overload_cast<const Vector<Vector<Segment3d<Int>>>&>(&CirDB::pathStats, py::const_))
    .def("net_res", &CirDB::netRes)
    .def("path_res", py::overload_cast<const Pin&, const Pin&>(&CirDB::pathRes, py::const_))

    .def("show_info", &CirDB::showInfo)
    .def("show_place_hpwl", &CirDB::showPlaceHpwl)
    .def("show_route_wl", &CirDB::showRouteWL)
    .def("show_net_segs", &CirDB::showNetSegs);

}


void initDbAPI(py::module& m)
{
  // enums
  initLayerTypeEnum(m);
  initTrackUseEnum(m);
  initPinDirEnum(m);
  initPinUseEnum(m);
  initPinShapeEnum(m);
  initIOPinLocEnum(m);
  initOrient2dEnum(m);
  initViaTypeEnum(m);
  initNetTypeEnum(m);
  initNetIOLocEnum(m);
  initSymAxisEnum(m);
  initSymPartEnum(m);
  initArrayPatternEnum(m);
  initArrayPartEnum(m);
  initExtTypeEnum(m);
  initDirection2dEnum(m);
  initDirection3dEnum(m);
  initRoutePrefEnum(m);
  initDrvEnum(m);

  // cir elements
  initLayerAPI(m);
  initPlaceGridAPI(m);
  initRouteGridAPI(m);
  initPrimitiveAPI(m);
  initDevMapAPI(m);
  initPinAPI(m);
  initObsAPI(m);
  initCellAPI(m);
  initViaAPI(m);
  initTopoTreeAPI(m);
  initRoutableAPI(m);
  initNetAPI(m);
  initRegionAPI(m);

  // constraints
  initPrePlaceCstrAPI(m);
  initPlaceSymCstrAPI(m);
  initPlaceArrayCstrAPI(m);
  initPlaceClusterCstrAPI(m);
  initPlaceExtCstrAPI(m);
  initPlaceEdgeDistCstrAPI(m);
  initPlaceOrderCstrAPI(m);
  initPlaceAlignCstrAPI(m);
  initPlaceDisjointCstrAPI(m);
  initPlaceRowCstrAPI(m);
  initRouteSymCstrAPI(m);
  initRoutePathMatchCstrAPI(m);
  initNetPrioCstrAPI(m);
  

  initCirAPI(m);
}

//////////////////////////////////
//        Parser Api            //
//////////////////////////////////
void initParserAPI(py::module& m)
{
  py::class_<Parser>(m, "parser")
    .def(py::init<CirDB&>())
    .def("parse", &Parser::parse)
    .def("parse_place", &Parser::parsePlace)
    .def("parse_route_gds", &Parser::parseRouteGds, py::arg("const String&"), py::arg("const Vector<Int>&") = Vector<Int>());
}

//////////////////////////////////
//        Placer Api            //
//////////////////////////////////
void initPlaceMgrAPI(py::module& m) 
{
  py::class_<PlaceMgr>(m, "place_mgr")
    .def(py::init<CirDB&>())
    .def("solve_smt", &PlaceMgr::solveSMT)
    .def("gen_edge_cells", &PlaceMgr::genEdgeCells)
    .def("gen_fill_cells", &PlaceMgr::genFillCells)
    .def("set_sym_cell_orients", &PlaceMgr::setSymCellOrients)
    .def("set_row_cell_orients", &PlaceMgr::setRowCellOrients);
}

void initPlaceSMTAPI(py::module& m)
{
  py::class_<SmtPlacer>(m, "place_smt")
    .def(py::init<CirDB&>())
    .def("solve", &SmtPlacer::solve)
    .def("set_ar", &SmtPlacer::setAR)
    //.def("set_fc_infl", &SmtPlacer::setFcInfl)
    //.def("set_fc_shrink", &SmtPlacer::setFcShrink)
    //.def("set_fc_hpwl_opt", &SmtPlacer::setFcHpwlOpt)
    .def("set_fc_util", &SmtPlacer::setFcUtil)
    .def("set_fc_area_expand", &SmtPlacer::setFcAreaExpand);
    //TODO 
}


void initPlacerAPI(py::module& m)
{
  initPlaceMgrAPI(m);
  initPlaceSMTAPI(m);
}

//////////////////////////////////
//        Router Api            //
//////////////////////////////////
void initRouteMgrAPI(py::module& m)
{
  py::class_<RouteMgr>(m, "route_mgr")
    .def(py::init<CirDB&>())
    .def("gen_pg", &RouteMgr::genPg)
    .def("gen_pg_grid", &RouteMgr::genPgGrid)
    .def("connectPgGrid", &RouteMgr::connectPgGrid);
}

void initRouteGrMgrAPI(py::module& m) 
{
  py::class_<RouteGrMgr>(m, "route_gr_mgr")
    .def(py::init<CirDB&>());
}

void initRouteDrMgrAPI(py::module& m) 
{
  py::class_<RouteDrMgr::GridEdge>(m, "grid_edge")
    .def(py::init<const Int, const Int>())
    .def_readwrite("u", &RouteDrMgr::GridEdge::u)
    .def_readwrite("v", &RouteDrMgr::GridEdge::v)
    .def("adj_node_idx", &RouteDrMgr::GridEdge::adjNodeIdx)
    .def(py::self == py::self)
    .def("__hash__", [](const RouteDrMgr::GridEdge& e) { return hash_value(e); });

  py::class_<RouteDrMgr>(m, "route_dr_mgr")
    .def(py::init<CirDB&>())
    .def("solve_ps", &RouteDrMgr::solvePS)
    .def("init", &RouteDrMgr::init, py::arg("const bool") = false, py::arg("const Int") = 14)
    .def("compute_pin_acs", &RouteDrMgr::computePinAcs)
    .def("compute_box_acs", &RouteDrMgr::computeBoxAcs)
    .def("compute_net_bbox", &RouteDrMgr::computeNetBbox)
    .def("set_net_sym_cstrs", &RouteDrMgr::setNetSymCstrs)
    .def("set_net_path_matching_cstrs", &RouteDrMgr::setNetPathMatchingCstrs)
    .def("find_sym_axis", &RouteDrMgr::findSymAxis)
    .def("find_self_sym_axis", &RouteDrMgr::findSelfSymAxis)
    .def("deg_pin_sym", &RouteDrMgr::degPinSym)
    .def("deg_pin_self_sym", &RouteDrMgr::degPinSelfSym)
    .def("is_cross_sym", &RouteDrMgr::isCrossSym)
    .def("is_cross_net", &RouteDrMgr::isCrossNet)
    .def("is_part_a_pin", &RouteDrMgr::isPartAPin)
    .def("is_part_b_pin", &RouteDrMgr::isPartBPin)
    .def("has_sym_pin", &RouteDrMgr::hasSymPin)
    .def("has_sym_pin_x", &RouteDrMgr::hasSymPinX)
    .def("has_sym_pin_y", &RouteDrMgr::hasSymPinY)
    .def("sym_pin", &RouteDrMgr::symPin, py::return_value_policy::reference_internal)
    .def("add_pin_shapes", &RouteDrMgr::addPinShapes)
    .def("gen_io_pin_shapes", &RouteDrMgr::genIOPinShapes)
    .def("grid_node_const", &RouteDrMgr::gridNode, py::return_value_policy::reference_internal)
    .def("grid_edge_const", &RouteDrMgr::gridEdge, py::return_value_policy::reference_internal)
    .def("grid_node_idx", &RouteDrMgr::gridNodeIdx)
    .def("grid_edge_idx", py::overload_cast<const RouteDrMgr::GridEdge&>(&RouteDrMgr::gridEdgeIdx, py::const_))
    .def("grid_edge_idx", py::overload_cast<Int, Int>(&RouteDrMgr::gridEdgeIdx, py::const_))
    .def("v_grid_edge_ids", py::overload_cast<const Box<Int>&, const Int, Vector<Int>&>(&RouteDrMgr::vGridEdgeIds, py::const_))
    .def("v_grid_edge_ids", py::overload_cast<const Segment3d<Int>&, Vector<Int>&>(&RouteDrMgr::vGridEdgeIds, py::const_))
    .def("grid_edge_to_seg", py::overload_cast<const RouteDrMgr::GridEdge&>(&RouteDrMgr::gridEdge2Seg, py::const_))
    .def("grid_edge_to_seg", py::overload_cast<const Int>(&RouteDrMgr::gridEdge2Seg, py::const_))
    .def("adj_node_const", &RouteDrMgr::adjNode, py::return_value_policy::reference_internal)
    .def("v_adj_edge_ids", py::overload_cast<const RouteDrMgr::GridNode&>(&RouteDrMgr::vAdjEdgeIds, py::const_), py::return_value_policy::reference_internal)
    .def("v_adj_edge_ids", py::overload_cast<const Int>(&RouteDrMgr::vAdjEdgeIds, py::const_), py::return_value_policy::reference_internal)
    .def("has_node", &RouteDrMgr::hasNode)
    .def("has_edge", &RouteDrMgr::hasEdge)
    .def("num_grid_nodes", &RouteDrMgr::numGridNodes)
    .def("num_grid_edges", &RouteDrMgr::numGridEdges)

    //.def("rl_pin", py::overload_cast<const Int>(&RouteDrMgr::rlPin), py::return_value_policy::reference_internal)
    //.def("rl_pin", py::overload_cast<const String&>(&RouteDrMgr::rlPin), py::return_value_policy::reference_internal)
    //.def("rl_pin", py::overload_cast<const Int>(&RouteDrMgr::rlPin, py::const_), py::return_value_policy::reference_internal)
    //.def("rl_pin", py::overload_cast<const String&>(&RouteDrMgr::rlPin, py::const_), py::return_value_policy::reference_internal)
    //.def("num_rl_pins", &RouteDrMgr::numRLPins)
    //.def("has_rl_pin", &RouteDrMgr::hasRLPin)

    .def("path_match_bbox", py::overload_cast<>(&RouteDrMgr::pathMatchBbox, py::const_))
    .def("path_match_bbox", py::overload_cast<const RoutePathMatchCstr&>(&RouteDrMgr::pathMatchBbox, py::const_))
    //.def("insert_rl_via", py::overload_cast<Net&, const Box<Int>&, const Int, const Int, const Int, const Orient2dE>(&RouteDrMgr::insertRLVia))
    //.def("insert_rl_via", py::overload_cast<Net&, const Point3d<Int>&, const Orient2dE>(&RouteDrMgr::insertRLVia))
    //.def("clear_rl_pins", &RouteDrMgr::clearRLPins)
    .def("pin_signal_track_pt", &RouteDrMgr::pinSignalTrackPt)
    .def("signal_track_pt", &RouteDrMgr::signalTrackPt)
    .def("num_signal_tracks", &RouteDrMgr::numSignalTracks)
    .def("num_signal_tracks_x", &RouteDrMgr::numXSignalTracks)
    .def("num_signal_tracks_y", &RouteDrMgr::numYSignalTracks)
    //.def("rl_data_net_names", py::overload_cast<>(&RouteDrMgr::rlDataNetNames, py::const_))
    //.def("rl_data", py::overload_cast<const Box<Int>&, const Net&>(&RouteDrMgr::rlData, py::const_))
    //.def("rl_data", py::overload_cast<const Box<Int>&>(&RouteDrMgr::rlData, py::const_))

    .def("reset_routing", &RouteDrMgr::resetRouting);

}

void initDrPsRouterAPI(py::module& m)
{
  py::class_<DrPsRouter>(m, "route_dr_ps")
    .def(py::init<RouteDrMgr&, const Int>(), py::arg("RouteDrMgr&"), py::arg("const Int") = 1)
    .def("solve", &DrPsRouter::solve, py::arg("const bool") = false, py::arg("const bool") = false)
    .def("history", py::overload_cast<const Int>(&DrPsRouter::history, py::const_))
    .def("history", py::overload_cast<const Segment3d<Int>&>(&DrPsRouter::history, py::const_))
    .def("history", py::overload_cast<const Segment3d<Int>&, const Int, const Real>(&DrPsRouter::history, py::const_))
    .def("neighbor_history", &DrPsRouter::neighborHistory)
    .def("add_history", py::overload_cast<const Int, const Real>(&DrPsRouter::addHistory))
    .def("add_history", py::overload_cast<const Segment3d<Int>&, const Real>(&DrPsRouter::addHistory))
    .def("decay_history", py::overload_cast<const Int, const Real>(&DrPsRouter::decayHistory))
    .def("decay_history", py::overload_cast<const Segment3d<Int>&, const Real>(&DrPsRouter::decayHistory))
    .def("clear_history", py::overload_cast<const Int>(&DrPsRouter::clearHistory))
    .def("clear_history", py::overload_cast<const Segment3d<Int>&>(&DrPsRouter::clearHistory))
    .def("reset_history", &DrPsRouter::resetHistory)
    .def("construct_net_routables", &DrPsRouter::constructNetRoutables)
    .def("construct_net_routables_normal", &DrPsRouter::constructNormalNetRoutables)
    .def("construct_net_routables_sym", &DrPsRouter::constructSymNetRoutables)
    .def("construct_net_routables_self_sym", &DrPsRouter::constructSelfSymNetRoutables)
    .def("construct_net_routables_cross_sym", &DrPsRouter::constructCrossSymNetRoutables)
    .def("add_unrouted_net", &DrPsRouter::addUnroutedNet)
    .def("add_unrouted_nets", &DrPsRouter::addUnroutedNets)
    .def("run_nrr", py::overload_cast<const Vector<Int>&, const bool>(&DrPsRouter::runNrr), py::arg("const Vector<Int>&"), py::arg("const bool") = false)
    .def("run_nrr", py::overload_cast<const bool, const NetTypeE>(&DrPsRouter::runNrr))
    .def("run_nrr_core", py::overload_cast<const NetTypeE, const bool, const bool, const Int, const Int>(&DrPsRouter::runNrrCore))
    .def("run_nrr_core", py::overload_cast<const Vector<Int>&, const bool, const bool, const Int, const Int>(&DrPsRouter::runNrrCore))
    .def("has_unrouted_net", &DrPsRouter::hasUnRoutedNet)
    .def("next_unrouted_net_idx", &DrPsRouter::nextUnRoutedNetIdx)
    .def("route", &DrPsRouter::route)
    .def("check_drc", py::overload_cast<const Vector<Int>&, const bool>(&DrPsRouter::checkDRC, py::const_))
    .def("check_drc", py::overload_cast<const bool, const NetTypeE>(&DrPsRouter::checkDRC, py::const_))
    .def("check_net_drc", &DrPsRouter::checkNetDRC)
    .def("ripup", &DrPsRouter::ripup)
    .def("ripup_segment", &DrPsRouter::ripupSegment)
    .def("ripup_partial", &DrPsRouter::ripupPartial)
    .def("ripup_segment_refine", &DrPsRouter::ripupSegmentNRefine)
    .def("ripup_partial_refine", &DrPsRouter::ripupPartialNRefine)
    .def("ripup_segment_and_floating", &DrPsRouter::ripupSegmentNFloating)
    .def("ripup_partial_and_floating", &DrPsRouter::ripupPartialNFloating)
    .def("ripup_floating", &DrPsRouter::ripupFloating)
    .def("refine_net", &DrPsRouter::refineNet)
    .def("reset_net_wires", &DrPsRouter::resetNetWires)
    .def("reset_net_status", &DrPsRouter::resetNetStatus)
    //.def("query_seg_wires", &DrPsRouter::querySegWires)
    //.def("ripup_segment_impl", &DrPsRouter::ripupSegmentImpl)
    .def("segs_to_wires", py::overload_cast<const Vector<Segment3d<Int>>&, Vector<Pair<Box<Int>, Int>>&>(&DrPsRouter::segs2Wires, py::const_))
    .def("segs_to_wires", py::overload_cast<const Vector<Segment3d<Int>>&, const Vector<TopoTree::SegData>&, Vector<Pair<Box<Int>, Int>>&>(&DrPsRouter::segs2Wires, py::const_))
    .def("seg_to_wires", py::overload_cast<const Segment3d<Int>&, Vector<Pair<Box<Int>, Int>>&>(&DrPsRouter::seg2Wires, py::const_))
    .def("seg_to_wires", py::overload_cast<const Segment3d<Int>&, const TopoTree::SegData&, Vector<Pair<Box<Int>, Int>>&>(&DrPsRouter::seg2Wires, py::const_))
    .def("merge_wires", &DrPsRouter::mergeWires)
    .def("query_net_seg_wires", &DrPsRouter::queryNetSegWires)
    .def("query_topo_wires", py::overload_cast<const Net&, const TopoTree&, Vector<Pair<Box<Int>, Int>>&>(&DrPsRouter::queryTopoWires, py::const_))
    .def("query_topo_comp_wires", py::overload_cast<const Net&, const TopoTree&, const Int, Vector<Pair<Box<Int>, Int>>&>(&DrPsRouter::queryTopoCompWires, py::const_))
    .def("query_topo_segs_wires", py::overload_cast<const Net&, const Vector<Segment3d<Int>>&, Vector<Pair<Box<Int>, Int>>&>(&DrPsRouter::queryTopoSegsWires, py::const_))
    .def("topo_comp_to_acs_pts", py::overload_cast<const TopoTree&, const Int, Vector<Point3d<Int>>&>(&DrPsRouter::topoComp2AcsPts, py::const_))

    .def("check_failed", &DrPsRouter::checkFailed);

}

void initPostRouteAPI(py::module& m)
{
  py::class_<PostRoute>(m, "post_route")
    .def(py::init<CirDB&>())
    .def("patch", py::overload_cast<>(&PostRoute::patch))
    .def("patch", py::overload_cast<Region&>(&PostRoute::patch));

}

void initRouterAPI(py::module& m)
{
  initRouteMgrAPI(m);
  initRouteGrMgrAPI(m);
  initRouteDrMgrAPI(m);
  initDrPsRouterAPI(m);
  initPostRouteAPI(m);
}

//////////////////////////////////
//        DRC Api               //
//////////////////////////////////
void initDrcMgrAPI(py::module& m)
{
  py::class_<DrcMgr>(m, "drc_mgr")
    .def(py::init<CirDB&>())
    .def("check_wire_valid_width", &DrcMgr::checkWireValidWidth)
    .def("check_wire_valid_length", &DrcMgr::checkWireValidLength)
    .def("check_wire_metal_spacing", &DrcMgr::checkWireMetalSpacing)
    .def("check_wire_metal_prl_spacing", &DrcMgr::checkWireMetalPrlSpacing)
    .def("check_wire_metal_eol_spacing", &DrcMgr::checkWireMetalEolSpacing)
    .def("check_wire_cut_spacing", &DrcMgr::checkWireCutSpacing)
    .def("check_wire_min_area", &DrcMgr::checkWireMinArea)
    .def("check_via_spacing", &DrcMgr::checkViaSpacing)
    .def("check_wire_min_step", py::overload_cast<const Int, const Box<Int>&>(&DrcMgr::checkWireMinStep, py::const_))
    .def("check_wire", &DrcMgr::checkWire)
    .def("check_net", &DrcMgr::checkNet)
    .def("fit_wire_to_valid_length", &DrcMgr::fitWire2ValidLength);
}

//////////////////////////////////
//        Writer Api            //
//////////////////////////////////
void initWriterAPI(py::module& m)
{
  py::class_<Writer>(m, "writer")
    .def(py::init<CirDB&>())
    .def("to_gv", &Writer::toGv)
    .def("to_placement", &Writer::toPlace)
    .def("to_icc_tcl", &Writer::toIccTcl);
}

//////////////////////////////////
//          Geo Api             //
//////////////////////////////////
template<typename T>
void initPointAPI(py::module& m, const String& typeName)
{
  const String className = String("point_") + typeName; 
  py::class_<Point<T>>(m, className.c_str())
    .def(py::init<>())
    .def(py::init<T, T>())
    .def(py::init<Point<T>>())
    .def(py::self == py::self)
    .def(py::self != py::self)
    .def(py::self < py::self)
    .def(py::self > py::self)
    .def(py::self <= py::self)
    .def(py::self >= py::self)
    .def(py::self + py::self)
    .def(py::self - py::self)
    .def(py::pickle(
            [](const Point<T>& p) {
              return py::make_tuple(p.x(), p.y());
            },
            [](py::tuple t) {
              return Point<T>(t[0].cast<T>(), t[1].cast<T>());
            }
                   ))
    .def("__hash__", [](const Point<T>& s) { return hash_value(s); })
    .def("set", &Point<T>::set)
    .def("set_x", &Point<T>::setX)
    .def("set_y", &Point<T>::setY)
    .def("set_xy", &Point<T>::setXY)
    .def("shift", &Point<T>::shift)
    .def("shift_x", &Point<T>::shiftX)
    .def("shift_y", &Point<T>::shiftY)
    .def("scale", py::overload_cast<const Real>(&Point<T>::scale))
    .def("scale", py::overload_cast<const Real, const Real>(&Point<T>::scale))
    .def("x", &Point<T>::x)
    .def("y", &Point<T>::y)
    .def("rotate_90", &Point<T>::rotate90)
    .def("rotate_180", &Point<T>::rotate180)
    .def("flip_x", &Point<T>::flipX)
    .def("flip_y", &Point<T>::flipY);
}

template<typename T>
void initPoint3dAPI(py::module& m, const String& typeName)
{
  const String className = String("point3d_") + typeName;
  py::class_<Point3d<T>>(m, className.c_str())
    .def(py::init<>())
    .def(py::init<T, T, T>())
    .def(py::self == py::self)
    .def(py::self != py::self)
    .def(py::self < py::self)
    .def(py::self > py::self)
    .def(py::self <= py::self)
    .def(py::self >= py::self)
    .def(py::self + py::self)
    .def(py::self - py::self)
    .def(py::pickle(
            [](const Point3d<T>& p) {
              return py::make_tuple(p.x(), p.y(), p.z());
            },
            [](py::tuple t) {
              return Point3d<T>(t[0].cast<T>(), t[1].cast<T>(), t[2].cast<T>());
            }
                   ))
    .def("__hash__", [](const Point3d<T>& s) { return hash_value(s); })
    .def("set", &Point3d<T>::set)
    .def("set_x", &Point3d<T>::setX)
    .def("set_y", &Point3d<T>::setY)
    .def("set_z", &Point3d<T>::setZ)
    .def("set_xyz", &Point3d<T>::setXYZ)
    .def("shift", &Point3d<T>::shift)
    .def("shift_x", &Point3d<T>::shiftX)
    .def("shift_y", &Point3d<T>::shiftY)
    .def("shift_z", &Point3d<T>::shiftZ)
    .def("shift_xyz", &Point3d<T>::shiftXYZ)
    .def("x", &Point3d<T>::x)
    .def("y", &Point3d<T>::y)
    .def("z", &Point3d<T>::z)
    .def("flip_x", &Point3d<T>::flipX)
    .def("flip_y", &Point3d<T>::flipY)
    .def("flip_z", &Point3d<T>::flipZ);
}

template<typename T>
void initSegmentAPI(py::module& m, const String& typeName)
{
  const String className = String("segment_") + typeName;
  py::class_<Segment<T>>(m, className.c_str())
    .def(py::init<>())
    .def(py::init<T, T, T, T>())
    .def(py::init<const Point<T>&, const Point<T>&>())
    .def(py::self == py::self)
    .def(py::self != py::self)
    .def(py::self < py::self)
    .def(py::self > py::self)
    .def(py::self <= py::self)
    .def(py::self >= py::self)
    .def(py::pickle(
            [](const Segment<T>& s) {
              return py::make_tuple(s.xl(), s.yl(), s.xh(), s.yh());
            },
            [](py::tuple t) {
              return Segment<T>(t[0].cast<T>(), t[1].cast<T>(), t[2].cast<T>(), t[3].cast<T>());
            }
                   ))
    .def("__hash__", [](const Segment<T>& s) { return hash_value(s); })
    .def("p0", py::overload_cast<>(&Segment<T>::p0), py::return_value_policy::reference_internal)
    .def("p0_const", py::overload_cast<>(&Segment<T>::p0, py::const_), py::return_value_policy::reference_internal)
    .def("p1", py::overload_cast<>(&Segment<T>::p1), py::return_value_policy::reference_internal)
    .def("p1_const", py::overload_cast<>(&Segment<T>::p1, py::const_), py::return_value_policy::reference_internal)
    .def("length", &Segment<T>::length)
    .def("xl", &Segment<T>::xl)
    .def("xh", &Segment<T>::xh)
    .def("yl", &Segment<T>::yl)
    .def("yh", &Segment<T>::yh)
    .def("center_x", &Segment<T>::centerX)
    .def("center_y", &Segment<T>::centerY)
    .def("center", &Segment<T>::center)
    .def("min_corner", &Segment<T>::min_corner)
    .def("max_corner", &Segment<T>::max_corner)
    .def("is_hor", &Segment<T>::bHor)
    .def("is_ver", &Segment<T>::bVer)
    .def("is_90", &Segment<T>::b90)
    .def("shift_x", &Segment<T>::shiftX)
    .def("shift_y", &Segment<T>::shiftY)
    .def("shift_xy", &Segment<T>::shiftXY)
    .def("is_connected", &Segment<T>::bConnect);
}

template<typename T>
void initSegment3dAPI(py::module& m, const String& typeName)
{
  const String className = String("segment3d_") + typeName;
  py::class_<Segment3d<T>>(m, className.c_str())
    .def(py::init<>())
    .def(py::init<T, T, T, T, T, T>())
    .def(py::init<const Point3d<T>&, const Point3d<T>&>())
    .def(py::self == py::self)
    .def(py::self != py::self)
    .def(py::self < py::self)
    .def(py::self > py::self)
    .def(py::self <= py::self)
    .def(py::self >= py::self)
    .def(py::pickle(
            [](const Segment3d<T>& s) {
              return py::make_tuple(s.xl(), s.yl(), s.zl(), s.xh(), s.yh(), s.zh());
            },
            [](py::tuple t) {
              return Segment3d<T>(t[0].cast<T>(), t[1].cast<T>(), t[2].cast<T>(), t[3].cast<T>(), t[4].cast<T>(), t[5].cast<T>());
            }
                   ))
    .def("__hash__", [](const Segment3d<T>& s) { return hash_value(s); })
    .def("p0", py::overload_cast<>(&Segment3d<T>::p0), py::return_value_policy::reference_internal)
    .def("p0", py::overload_cast<>(&Segment3d<T>::p0, py::const_), py::return_value_policy::reference_internal)
    .def("p1", py::overload_cast<>(&Segment3d<T>::p1), py::return_value_policy::reference_internal)
    .def("p1", py::overload_cast<>(&Segment3d<T>::p1, py::const_), py::return_value_policy::reference_internal)
    .def("length", &Segment3d<T>::length)
    .def("xl", &Segment3d<T>::xl)
    .def("xh", &Segment3d<T>::xh)
    .def("yl", &Segment3d<T>::yl)
    .def("yh", &Segment3d<T>::yh)
    .def("zl", &Segment3d<T>::zl)
    .def("zh", &Segment3d<T>::zh)
    .def("center_x", &Segment3d<T>::centerX)
    .def("center_y", &Segment3d<T>::centerY)
    .def("center_z", &Segment3d<T>::centerZ)
    .def("center", &Segment3d<T>::center)
    .def("min_corner", &Segment3d<T>::min_corner)
    .def("max_corner", &Segment3d<T>::max_corner)
    .def("is_hor", &Segment3d<T>::bHor)
    .def("is_ver", &Segment3d<T>::bVer)
    .def("is_via", &Segment3d<T>::bVia)
    .def("is_90", &Segment3d<T>::b90)
    .def("is_connected", &Segment3d<T>::bConnect);
}

template<typename T>
void initBoxAPI(py::module& m, const String& typeName)
{
  const String className = String("box_") + typeName;
  py::class_<Box<T>>(m, className.c_str())
    .def(py::init<>())
    .def(py::init<T, T, T, T>())
    .def(py::init<const Point<T>&, const Point<T>&>())
    .def(py::self == py::self)
    .def(py::self != py::self)
    .def(py::self < py::self)
    .def(py::self > py::self)
    .def(py::self <= py::self)
    .def(py::self >= py::self)
    .def(py::pickle(
            [](const Box<T>& b) {
              return py::make_tuple(b.xl(), b.yl(), b.xh(), b.yh());
            },
            [](py::tuple t) {
              return Box<T>(t[0].cast<T>(), t[1].cast<T>(), t[2].cast<T>(), t[3].cast<T>());
            }
                   ))
    .def("__hash__", [](const Box<T>& b) { return hash_value(b); })
    .def("set_xl", &Box<T>::setXL)
    .def("set_xh", &Box<T>::setXH)
    .def("set_yl", &Box<T>::setYL)
    .def("set_yh", &Box<T>::setYH)
    .def("set", py::overload_cast<T, T, T, T>(&Box<T>::set))
    .def("set", py::overload_cast<const Box<T>&>(&Box<T>::set))
    .def("xl", &Box<T>::xl)
    .def("xh", &Box<T>::xh)
    .def("yl", &Box<T>::yl)
    .def("yh", &Box<T>::yh)
    .def("width", &Box<T>::width)
    .def("height", &Box<T>::height)
    .def("center_x", &Box<T>::centerX)
    .def("center_y", &Box<T>::centerY)
    .def("hpwl", &Box<T>::hpwl)
    .def("perimeter", &Box<T>::perimeter)
    .def("area", &Box<T>::area)
    .def("bl", py::overload_cast<>(&Box<T>::bl), py::return_value_policy::reference_internal)
    .def("bl_const", py::overload_cast<>(&Box<T>::bl, py::const_), py::return_value_policy::reference_internal)
    .def("tr", py::overload_cast<>(&Box<T>::tr), py::return_value_policy::reference_internal)
    .def("tr_const", py::overload_cast<>(&Box<T>::tr, py::const_), py::return_value_policy::reference_internal)
    .def("min_corner", py::overload_cast<>(&Box<T>::min_corner), py::return_value_policy::reference_internal)
    .def("min_corner_const", py::overload_cast<>(&Box<T>::min_corner, py::const_), py::return_value_policy::reference_internal)
    .def("max_corner", py::overload_cast<>(&Box<T>::max_corner), py::return_value_policy::reference_internal)
    .def("max_corner_const", py::overload_cast<>(&Box<T>::max_corner, py::const_), py::return_value_policy::reference_internal)
    .def("center", &Box<T>::center)
    .def("shift_x", &Box<T>::shiftX)
    .def("shift_y", &Box<T>::shiftY)
    .def("shift", &Box<T>::shift)
    .def("rotate_90", &Box<T>::rotate90)
    .def("rotate_180", &Box<T>::rotate180)
    .def("flip_x", &Box<T>::flipX)
    .def("flip_y", &Box<T>::flipY)
    .def("expand", py::overload_cast<T>(&Box<T>::expand))
    .def("expand", py::overload_cast<T, const int>(&Box<T>::expand))
    .def("expand_x", &Box<T>::expandX)
    .def("expand_y", &Box<T>::expandY)
    .def("shrink", &Box<T>::shrink)
    .def("shrink_x", &Box<T>::shrinkX)
    .def("shrink_y", &Box<T>::shrinkY);
}

void initSpatialEnumAPI(py::module& m)
{
  py::enum_<spatial::QueryType>(m, "spatial_query_type_e")
    .value("contains", spatial::QueryType::contains)
    .value("covered_by", spatial::QueryType::covered_by)
    .value("covers", spatial::QueryType::covers)
    .value("disjoint", spatial::QueryType::disjoint)
    .value("intersects", spatial::QueryType::intersects)
    .value("overlaps", spatial::QueryType::overlaps)
    .value("within", spatial::QueryType::within);

}

template<typename T>
void initSpatialAPI(py::module& m, const String& typeName)
{
  
  py::class_<Spatial<T>>(m, String("spatial_" + typeName).c_str())
    .def(py::init<>())
    .def(py::init<const Spatial<T>&>())
    .def("__len__", [] (const Spatial<T>& s) { return s.size(); })
    .def("__iter__", [] (const Spatial<T>& s) { return py::make_iterator(s.begin(), s.end()); }, py::keep_alive<0, 1>())
    .def("empty", &Spatial<T>::empty)
    .def("size", &Spatial<T>::size)
    .def("clear", &Spatial<T>::clear)
    .def("insert", py::overload_cast<const Box<T>&>(static_cast<void (Spatial<T>::*)(const Box<T>&)>(&Spatial<T>::insert)))
    .def("insert", py::overload_cast<const Point<T>&, const Point<T>&>(static_cast<void (Spatial<T>::*)(const Point<T>&, const Point<T>&)>(&Spatial<T>::insert)))
    .def("erase", py::overload_cast<const Box<T>&>(static_cast<bool (Spatial<T>::*)(const Box<T>&)>(&Spatial<T>::erase)))
    .def("erase", py::overload_cast<const Point<T>&, const Point<T>&>(static_cast<bool (Spatial<T>::*)(const Point<T>&, const Point<T>&)>(&Spatial<T>::erase)))
    .def("deep_erase", py::overload_cast<const Box<T>&>(static_cast<bool (Spatial<T>::*)(const Box<T>&)>(&Spatial<T>::deepErase)))
    .def("deep_erase", py::overload_cast<const Point<T>&, const Point<T>&>(static_cast<bool (Spatial<T>::*)(const Point<T>&, const Point<T>&)>(&Spatial<T>::deepErase)))
    .def("query", py::overload_cast<const Box<T>&, Vector<Box<T>>&, spatial::QueryType>(static_cast<void (Spatial<T>::*)(const Box<T>&, Vector<Box<T>>&, spatial::QueryType) const>(&Spatial<T>::query), py::const_), py::arg("const Box<T>&"), py::arg("Vector<Box<T>>&"), py::arg("spatial::QueryType") = spatial::QueryType::intersects)
    .def("query", py::overload_cast<const Point<T>&, const Point<T>&, Vector<Box<T>>&, spatial::QueryType>(static_cast<void (Spatial<T>::*)(const Point<T>&, const Point<T>&, Vector<Box<T>>&, spatial::QueryType) const>(&Spatial<T>::query), py::const_), py::arg("const Point<T>&"), py::arg("const Point<T>&"), py::arg("Vector<Box<T>>&"), py::arg("spatial::QueryType") = spatial::QueryType::intersects)
    .def("exist", py::overload_cast<const Box<T>&, spatial::QueryType>(static_cast<bool (Spatial<T>::*)(const Box<T>&, spatial::QueryType) const>(&Spatial<T>::exist), py::const_), py::arg("const Box<T>&"), py::arg("spatial::QueryType") = spatial::QueryType::intersects)
    .def("exist", py::overload_cast<const Point<T>&, const Point<T>&, spatial::QueryType>(static_cast<bool (Spatial<T>::*)(const Point<T>&, const Point<T>&, spatial::QueryType) const>(&Spatial<T>::exist), py::const_), py::arg("const Point<T>&"), py::arg("const Point<T>&"), py::arg("spatial::QueryType") = spatial::QueryType::intersects)
    .def("nearest_search", py::overload_cast<const Point<T>&, const Int, Vector<Box<T>>&>(static_cast<void (Spatial<T>::*)(const Point<T>&, const Int, Vector<Box<T>>&) const>(&Spatial<T>::nearestSearch), py::const_))
    .def("nearest_search", py::overload_cast<const Box<T>&, const Int, Vector<Box<T>>&>(static_cast<void (Spatial<T>::*)(const Box<T>&, const Int, Vector<Box<T>>&) const>(&Spatial<T>::nearestSearch), py::const_));

}

template<typename T, typename V>
void initSpatialMapAPI(py::module& m, const String& tName, const String& vName)
{
  py::class_<SpatialMap<T, V>>(m, String("spatial_map_" + tName + "_" + vName).c_str())
    .def(py::init<>())
    .def(py::init<const SpatialMap<T, V>&>())
    .def("__len__", [] (const SpatialMap<T, V>& s) { return s.size(); })
    .def("__iter__", [] (const SpatialMap<T, V>& s) { return py::make_iterator(s.begin(), s.end()); }, py::keep_alive<0, 1>())
    .def("empty", &SpatialMap<T, V>::empty)
    .def("size", &SpatialMap<T, V>::size)
    .def("clear", &SpatialMap<T, V>::clear)
    .def("insert", py::overload_cast<const spatial::b_value<T, V>&>(static_cast<void (SpatialMap<T, V>::*)(const spatial::b_value<T, V>&)>(&SpatialMap<T, V>::insert)))
    .def("insert", py::overload_cast<const Box<T>&, const V&>(static_cast<void (SpatialMap<T, V>::*)(const Box<T>&, const V&)>(&SpatialMap<T, V>::insert)))
    .def("insert", py::overload_cast<const Point<T>&, const Point<T>&, const V&>(static_cast<void (SpatialMap<T, V>::*)(const Point<T>&, const Point<T>&, const V&)>(&SpatialMap<T, V>::insert)))
    .def("erase", py::overload_cast<const spatial::b_value<T, V>&>(static_cast<bool (SpatialMap<T, V>::*)(const spatial::b_value<T, V>&)>(&SpatialMap<T, V>::erase)))
    .def("erase", py::overload_cast<const Box<T>&, const V&>(static_cast<bool (SpatialMap<T, V>::*)(const Box<T>&, const V&)>(&SpatialMap<T, V>::erase)))
    .def("erase", py::overload_cast<const Point<T>&, const Point<T>&, const V&>(static_cast<bool (SpatialMap<T, V>::*)(const Point<T>&, const Point<T>&, const V&)>(&SpatialMap<T, V>::erase)))
    .def("deep_erase", py::overload_cast<const spatial::b_value<T, V>&>(static_cast<bool (SpatialMap<T, V>::*)(const spatial::b_value<T, V>&)>(&SpatialMap<T, V>::deepErase)))
    .def("deep_erase", py::overload_cast<const Box<T>&, const V&>(static_cast<bool (SpatialMap<T, V>::*)(const Box<T>&, const V&)>(&SpatialMap<T, V>::deepErase)))
    .def("deep_erase", py::overload_cast<const Point<T>&, const Point<T>&, const V&>(static_cast<bool (SpatialMap<T, V>::*)(const Point<T>&, const Point<T>&, const V&)>(&SpatialMap<T, V>::deepErase)))
    .def("query", py::overload_cast<const Box<T>&, Vector<V>&, spatial::QueryType>(static_cast<void (SpatialMap<T, V>::*)(const Box<T>&,  Vector<V>&, spatial::QueryType) const>(&SpatialMap<T, V>::query), py::const_), py::arg("const Box<T>&"), py::arg("Vector<V>&"), py::arg("spatial::QueryType") = spatial::QueryType::intersects)
    .def("query", py::overload_cast<const Point<T>&, const Point<T>&, Vector<V>&, spatial::QueryType>(static_cast<void (SpatialMap<T, V>::*)(const Point<T>&, const Point<T>&, Vector<V>&, spatial::QueryType) const>(&SpatialMap<T, V>::query), py::const_), py::arg("const Point<T>&"), py::arg("const Point<T>&"), py::arg("Vector<V>&"), py::arg("spatial::QueryType") = spatial::QueryType::intersects)
    .def("query_box", py::overload_cast<const Box<T>&, Vector<Box<T>>&, spatial::QueryType>(static_cast<void (SpatialMap<T, V>::*)(const Box<T>&,  Vector<Box<T>>&, spatial::QueryType) const>(&SpatialMap<T, V>::queryBox), py::const_), py::arg("const Box<T>&"), py::arg("Vector<Box<T>>&"), py::arg("spatial::QueryType") = spatial::QueryType::intersects)
    .def("query_box", py::overload_cast<const Point<T>&, const Point<T>&, Vector<Box<T>>&, spatial::QueryType>(static_cast<void (SpatialMap<T, V>::*)(const Point<T>&, const Point<T>&, Vector<Box<T>>&, spatial::QueryType) const>(&SpatialMap<T, V>::queryBox), py::const_), py::arg("const Point<T>&"), py::arg("const Point<T>&"), py::arg("Vector<Box<T>>&"), py::arg("spatial::QueryType") = spatial::QueryType::intersects)
    .def("query_both", py::overload_cast<const Box<T>&, Vector<Pair<Box<T>, V>>&, spatial::QueryType>(static_cast<void (SpatialMap<T, V>::*)(const Box<T>&,  Vector<Pair<Box<T>, V>>&, spatial::QueryType) const>(&SpatialMap<T, V>::queryBoth), py::const_), py::arg("const Box<T>&"), py::arg("Vector<Pair<Box<T>, V>>&"), py::arg("spatial::QueryType") = spatial::QueryType::intersects)
    .def("query_both", py::overload_cast<const Point<T>&, const Point<T>&, Vector<Pair<Box<T>, V>>&, spatial::QueryType>(static_cast<void (SpatialMap<T, V>::*)(const Point<T>&, const Point<T>&, Vector<Pair<Box<T>, V>>&, spatial::QueryType) const>(&SpatialMap<T, V>::queryBoth), py::const_), py::arg("const Point<T>&"), py::arg("const Point<T>&"), py::arg("Vector<Pair<Box<T>, V>>&"), py::arg("spatial::QueryType") = spatial::QueryType::intersects)
    .def("exist", py::overload_cast<const Box<T>&, spatial::QueryType>(static_cast<bool (SpatialMap<T, V>::*)(const Box<T>&,  spatial::QueryType) const>(&SpatialMap<T, V>::exist), py::const_), py::arg("const Box<T>&"), py::arg("spatial::QueryType") = spatial::QueryType::intersects)
    .def("exist", py::overload_cast<const Point<T>&, const Point<T>&, spatial::QueryType>(static_cast<bool (SpatialMap<T, V>::*)(const Point<T>&, const Point<T>&, spatial::QueryType) const>(&SpatialMap<T, V>::exist), py::const_), py::arg("const Point<T>&"), py::arg("const Point<T>&"), py::arg("spatial::QueryType") = spatial::QueryType::intersects)
    .def("exist", py::overload_cast<const Box<T>&, const V&, spatial::QueryType>(static_cast<bool (SpatialMap<T, V>::*)(const Box<T>&, const V&, spatial::QueryType) const>(&SpatialMap<T, V>::exist), py::const_), py::arg("const Box<T>&"), py::arg("const V&"), py::arg("spatial::QueryType") = spatial::QueryType::intersects)
    .def("exist", py::overload_cast<const Point<T>&, const Point<T>&, const V&, spatial::QueryType>(static_cast<bool (SpatialMap<T, V>::*)(const Point<T>&, const Point<T>&, const V&, spatial::QueryType) const>(&SpatialMap<T, V>::exist), py::const_), py::arg("const Point<T>&"), py::arg("const Point<T>&"), py::arg("const V&"), py::arg("spatial::QueryType") = spatial::QueryType::intersects)
    .def("nearest_search", py::overload_cast<const Point<T>&, const Int, Vector<V>&>(static_cast<void (SpatialMap<T, V>::*)(const Point<T>&, const Int, Vector<V>&) const>(&SpatialMap<T, V>::nearestSearch), py::const_))
    .def("nearest_search", py::overload_cast<const Box<T>&, const Int, Vector<V>&>(static_cast<void (SpatialMap<T, V>::*)(const Box<T>&, const Int, Vector<V>&) const>(&SpatialMap<T, V>::nearestSearch), py::const_))
    .def("nearest_search_box", py::overload_cast<const Point<T>&, const Int, Vector<Box<T>>&>(static_cast<void (SpatialMap<T, V>::*)(const Point<T>&, const Int, Vector<Box<T>>&) const>(&SpatialMap<T, V>::nearestSearchBox), py::const_))
    .def("nearest_search_box", py::overload_cast<const Box<T>&, const Int, Vector<Box<T>>&>(static_cast<void (SpatialMap<T, V>::*)(const Box<T>&, const Int, Vector<Box<T>>&) const>(&SpatialMap<T, V>::nearestSearchBox), py::const_))
    .def("nearest_search_both", py::overload_cast<const Point<T>&, const Int, Vector<Pair<Box<T>, V>>&>(static_cast<void (SpatialMap<T, V>::*)(const Point<T>&, const Int, Vector<Pair<Box<T>, V>>&) const>(&SpatialMap<T, V>::nearestSearchBoth), py::const_))
    .def("nearest_search_both", py::overload_cast<const Box<T>&, const Int, Vector<Pair<Box<T>, V>>&>(static_cast<void (SpatialMap<T, V>::*)(const Box<T>&, const Int, Vector<Pair<Box<T>, V>>&) const>(&SpatialMap<T, V>::nearestSearchBoth), py::const_));

}

void initSpatial3dEnumAPI(py::module& m)
{
  py::enum_<spatial3d::QueryType>(m, "spatial3d_query_type_e")
    .value("contains", spatial3d::QueryType::contains)
    .value("covered_by", spatial3d::QueryType::covered_by)
    .value("covers", spatial3d::QueryType::covers)
    .value("disjoint", spatial3d::QueryType::disjoint)
    .value("intersects", spatial3d::QueryType::intersects)
    .value("overlaps", spatial3d::QueryType::overlaps)
    .value("within", spatial3d::QueryType::within);

}


template<typename T>
void initSpatial3dAPI(py::module& m, const String& typeName)
{
  const String className = String("spatial3d_") + typeName;
  py::class_<Spatial3d<T>>(m, className.c_str())
    .def(py::init<>())
    .def(py::init<const Spatial3d<T>&>())
    .def("__len__", [] (const Spatial3d<T>& s) { return s.size(); })
    .def("__iter__", [] (const Spatial3d<T>& s) { return py::make_iterator(s.begin(), s.end()); }, py::keep_alive<0, 1>())
    .def("empty", &Spatial3d<T>::empty)
    .def("size", &Spatial3d<T>::size)
    .def("clear", &Spatial3d<T>::clear)
    //.def("insert", py::overload_cast<const spatial3d::b_box<T>&>(static_cast<void (Spatial3d<T>::*)(const spatial3d::b_box<T>&)>(&Spatial3d<T>::insert)))
    .def("insert", py::overload_cast<const Point3d<T>&, const Point3d<T>&>(static_cast<void (Spatial3d<T>::*)(const Point3d<T>& ,const Point3d<T>&)>(&Spatial3d<T>::insert)))
    //.def("erase", py::overload_cast<const spatial3d::b_box<T>&>(static_cast<bool (Spatial3d<T>::*)(const spatial3d::b_box<T>&)>(&Spatial3d<T>::erase)))
    .def("erase", py::overload_cast<const Point3d<T>&, const Point3d<T>&>(static_cast<bool (Spatial3d<T>::*)(const Point3d<T>& ,const Point3d<T>&)>(&Spatial3d<T>::erase)))
    //.def("deep_erase", py::overload_cast<const spatial3d::b_box<T>&>(static_cast<bool (Spatial3d<T>::*)(const spatial3d::b_box<T>&)>(&Spatial3d<T>::deepErase)))
    .def("deep_erase", py::overload_cast<const Point3d<T>&, const Point3d<T>&>(static_cast<bool (Spatial3d<T>::*)(const Point3d<T>& ,const Point3d<T>&)>(&Spatial3d<T>::deepErase)))
    //.def("query", py::overload_cast<const Point3d<T>&, const Point3d<T>&, Vector<spatial3d::b_box<T>>&, spatial3d::QueryType>(static_cast<void (Spatial3d<T>::*)(const Point3d<T>&, const Point3d<T>&, Vector<spatial3d::b_box<T>>&, spatial3d::QueryType) const>(&Spatial3d<T>::query), py::const_), py::arg("const Point3d<T>&"), py::arg("const Point3d<T>&"), py::arg("Vector<spatial3d::b_box<T>>&"), py::arg("spatial3d::QueryType") = spatial3d::QueryType::intersects)
    .def("exist", py::overload_cast<const Point3d<T>&, const Point3d<T>&, spatial3d::QueryType>(static_cast<bool (Spatial3d<T>::*)(const Point3d<T>&, const Point3d<T>&, spatial3d::QueryType) const>(&Spatial3d<T>::exist), py::const_), py::arg("const Point3d<T>&"), py::arg("const Point3d<T>&"), py::arg("spatial3d::QueryType") = spatial3d::QueryType::intersects)
    //.def("nearest_search", py::overload_cast<const Point3d<T>&, const Int, Vector<spatial3d::b_box<T>>&>(static_cast<void (Spatial3d<T>::*)(const Point3d<T>&, const Int, Vector<spatial3d::b_box<T>>&) const>(&Spatial3d<T>::nearestSearch), py::const_))
    //.def("nearest_search", py::overload_cast<const Point3d<T>&, const Point3d<T>&, const Int, Vector<spatial3d::b_box<T>>&>(static_cast<void (Spatial3d<T>::*)(const Point3d<T>&, const Point3d<T>&, const Int, Vector<spatial3d::b_box<T>>&) const>(&Spatial3d<T>::nearestSearch), py::const_))
    ;
}

template<typename T, typename V>
void initSpatialMap3dAPI(py::module& m, const String& tName, const String& vName)
{
  py::class_<SpatialMap3d<T, V>>(m, String("spatial_map3d_" + tName + "_" + vName).c_str())
    .def(py::init<>())
    .def(py::init<const SpatialMap3d<T, V>&>())
    .def("__len__", [] (const SpatialMap3d<T, V>& s) { return s.size(); })
    .def("__iter__", [] (const SpatialMap3d<T, V>& s) { return py::make_iterator(s.begin(), s.end()); }, py::keep_alive<0, 1>())
    .def("empty", &SpatialMap3d<T, V>::empty)
    .def("size", &SpatialMap3d<T, V>::size)
    .def("clear", &SpatialMap3d<T, V>::clear)
    //.def("insert", py::overload_cast<const spatial3d::b_value<T, V>&>(static_cast<void (SpatialMap3d<T, V>::*)(const spatial3d::b_value<T, V>&)>(&SpatialMap3d<T, V>::insert)))
    .def("insert", py::overload_cast<const Point3d<T>&, const Point3d<T>&, const V&>(static_cast<void (SpatialMap3d<T, V>::*)(const Point3d<T>&, const Point3d<T>&, const V&)>(&SpatialMap3d<T, V>::insert)))
    //.def("erase", py::overload_cast<const spatial3d::b_value<T, V>&>(static_cast<bool (SpatialMap3d<T, V>::*)(const spatial3d::b_value<T, V>&)>(&SpatialMap3d<T, V>::erase)))
    .def("erase", py::overload_cast<const Point3d<T>&, const Point3d<T>&, const V&>(static_cast<bool (SpatialMap3d<T, V>::*)(const Point3d<T>&, const Point3d<T>&, const V&)>(&SpatialMap3d<T, V>::erase)))
    .def("query", py::overload_cast<const Point3d<T>&, const Point3d<T>&, Vector<V>&, spatial3d::QueryType>(static_cast<void (SpatialMap3d<T, V>::*)(const Point3d<T>&, const Point3d<T>&, Vector<V>&, spatial3d::QueryType) const>(&SpatialMap3d<T, V>::query), py::const_), py::arg("const Point3d<T>&"), py::arg("const Point3d<T>&"), py::arg("Vector<V>&"), py::arg("spatial3d::QueryType") = spatial3d::QueryType::intersects)
    //.def("query_box", py::overload_cast<const Point3d<T>&, const Point3d<T>&, Vector<spatial3d::b_box<T>>&, spatial3d::QueryType>(static_cast<void (SpatialMap3d<T, V>::*)(const Point3d<T>&, const Point3d<T>&, Vector<spatial3d::b_box<T>>&, spatial3d::QueryType) const>(&SpatialMap3d<T, V>::queryBox), py::const_), py::arg("const Point3d<T>&"), py::arg("const Point3d<T>&"), py::arg("Vector<spatial3d::b_box<T>>&"), py::arg("spatial3d::QueryType") = spatial3d::QueryType::intersects)
    //.def("query_both", py::overload_cast<const Point3d<T>&, const Point3d<T>&, Vector<spatial3d::b_value<T, V>>&, spatial3d::QueryType>(static_cast<void (SpatialMap3d<T, V>::*)(const Point3d<T>&, const Point3d<T>&, Vector<spatial3d::b_value<T, V>>&, spatial3d::QueryType) const>(&SpatialMap3d<T, V>::queryBoth), py::const_), py::arg("const Point3d<T>&"), py::arg("const Point3d<T>&"), py::arg("Vector<spatial3d::b_value<T, V>>&"), py::arg("spatial3d::QueryType") = spatial3d::QueryType::intersects)
    .def("exist", py::overload_cast<const Point3d<T>&, const Point3d<T>&, spatial3d::QueryType>(static_cast<bool (SpatialMap3d<T, V>::*)(const Point3d<T>&, const Point3d<T>&, spatial3d::QueryType) const>(&SpatialMap3d<T, V>::exist), py::const_), py::arg("const Point3d<T>&"), py::arg("const Point3d<T>&"), py::arg("spatial3d::QueryType") = spatial3d::QueryType::intersects)
    .def("exist", py::overload_cast<const Point3d<T>&, const Point3d<T>&, const V&, spatial3d::QueryType>(static_cast<bool (SpatialMap3d<T, V>::*)(const Point3d<T>&, const Point3d<T>&, const V&, spatial3d::QueryType) const>(&SpatialMap3d<T, V>::exist), py::const_), py::arg("const Point3d<T>&"), py::arg("const Point3d<T>&"), py::arg("const V&"), py::arg("spatial3d::QueryType") = spatial3d::QueryType::intersects)
    .def("nearest_search", py::overload_cast<const Point3d<T>&, const Int, Vector<V>&>(static_cast<void (SpatialMap3d<T, V>::*)(const Point3d<T>&, const Int, Vector<V>&) const>(&SpatialMap3d<T, V>::nearestSearch), py::const_))
    .def("nearest_search", py::overload_cast<const Point3d<T>&, const Point3d<T>&, const Int, Vector<V>&>(static_cast<void (SpatialMap3d<T, V>::*)(const Point3d<T>&, const Point3d<T>&, const Int, Vector<V>&) const>(&SpatialMap3d<T, V>::nearestSearch), py::const_))
    //.def("nearest_search_box", py::overload_cast<const Point3d<T>&, const Int, Vector<spatial3d::b_box<T>>&>(static_cast<void (SpatialMap3d<T, V>::*)(const Point3d<T>&, const Int, Vector<spatial3d::b_box<T>>&) const>(&SpatialMap3d<T, V>::nearestSearchBox), py::const_))
    //.def("nearest_search_box", py::overload_cast<const Point3d<T>&, const Point3d<T>&, const Int, Vector<spatial3d::b_box<T>>&>(static_cast<void (SpatialMap3d<T, V>::*)(const Point3d<T>&, const Point3d<T>&, const Int, Vector<spatial3d::b_box<T>>&) const>(&SpatialMap3d<T, V>::nearestSearchBox), py::const_))
    //.def("nearest_search_both", py::overload_cast<const Point3d<T>&, const Int, Vector<spatial3d::b_value<T, V>>&>(static_cast<void (SpatialMap3d<T, V>::*)(const Point3d<T>&, const Int, Vector<spatial3d::b_value<T, V>>&) const>(&SpatialMap3d<T, V>::nearestSearchBoth), py::const_))
    //.def("nearest_search_both", py::overload_cast<const Point3d<T>&, const Point3d<T>&, const Int, Vector<spatial3d::b_value<T, V>>&>(static_cast<void (SpatialMap3d<T, V>::*)(const Point3d<T>&, const Point3d<T>&, const Int, Vector<spatial3d::b_value<T, V>>&) const>(&SpatialMap3d<T, V>::nearestSearchBoth), py::const_))
    ;
}


void initGeoAPI(py::module& m)
{
  initBoxAPI<int>(m, "int"); // box_int
  initBoxAPI<double>(m, "float"); // box_float
  initPointAPI<int>(m, "int");
  initPointAPI<double>(m, "float");
  initPoint3dAPI<int>(m, "int");
  initPoint3dAPI<double>(m, "float");
  initSegmentAPI<int>(m, "int");
  initSegmentAPI<double>(m, "float");
  initSegment3dAPI<int>(m, "int");
  initSegment3dAPI<double>(m, "float");

  initSpatialEnumAPI(m);
  initSpatialAPI<int>(m, "int");
  initSpatialAPI<double>(m, "float");
  initSpatialMapAPI<int, int>(m, "int", "int");
  initSpatialMapAPI<int, double>(m, "int", "float");
  initSpatialMapAPI<double, int>(m, "float", "int");
  initSpatialMapAPI<double, double>(m, "float", "float");

  initSpatial3dEnumAPI(m);
  initSpatial3dAPI<int>(m, "int");
  initSpatial3dAPI<double>(m, "float");
  initSpatialMap3dAPI<int, int>(m, "int", "int");
  initSpatialMap3dAPI<int, double>(m, "int", "float");
  initSpatialMap3dAPI<double, int>(m, "float", "int");
  initSpatialMap3dAPI<double, double>(m, "float", "float");
}

template<typename T>
void initArray2dAPI(py::module& m, const String& typeName)
{
  const String className = String("array2d_") + typeName;
  py::class_<Array2d<T>>(m, className.c_str())
    .def(py::init<>())
    .def(py::init<const Int, const Int>())
    .def(py::init<const Int, const Int, const T&>())
    .def("num_x", &Array2d<T>::numX)
    .def("num_y", &Array2d<T>::numY)
    .def("size", &Array2d<T>::size)
    .def("at", py::overload_cast<const Int>(&Array2d<T>::at), py::return_value_policy::reference_internal)
    .def("at", py::overload_cast<const Int>(&Array2d<T>::at, py::const_), py::return_value_policy::reference_internal)
    .def("at", py::overload_cast<const Int, const Int>(&Array2d<T>::at), py::return_value_policy::reference_internal)
    .def("at", py::overload_cast<const Int, const Int>(&Array2d<T>::at, py::const_), py::return_value_policy::reference_internal)
    .def("clear", &Array2d<T>::clear)
    .def("resize", py::overload_cast<const Int, const Int>(&Array2d<T>::resize))
    .def("resize", py::overload_cast<const Int, const Int, const T&>(&Array2d<T>::resize))
    .def("set", &Array2d<T>::set);
}

template<typename T>
void initArray3dAPI(py::module& m, const String& typeName)
{
  const String className = String("array3d_") + typeName;
  py::class_<Array3d<T>>(m, className.c_str())
    .def(py::init<>())
    .def(py::init<const Int, const Int, const Int>())
    .def(py::init<const Int, const Int, const Int, const T&>())
    .def("num_x", &Array3d<T>::numX)
    .def("num_y", &Array3d<T>::numY)
    .def("num_z", &Array3d<T>::numZ)
    .def("size", &Array3d<T>::size)
    .def("at", py::overload_cast<const Int>(&Array3d<T>::at), py::return_value_policy::reference_internal)
    .def("at", py::overload_cast<const Int>(&Array3d<T>::at, py::const_), py::return_value_policy::reference_internal)
    .def("at", py::overload_cast<const Int, const Int, const Int>(&Array3d<T>::at), py::return_value_policy::reference_internal)
    .def("at", py::overload_cast<const Int, const Int, const Int>(&Array3d<T>::at, py::const_), py::return_value_policy::reference_internal)
    .def("clear", &Array3d<T>::clear)
    .def("resize", py::overload_cast<const Int, const Int, const Int>(&Array3d<T>::resize))
    .def("resize", py::overload_cast<const Int, const Int, const Int, const T&>(&Array3d<T>::resize))
    .def("set", &Array3d<T>::set);
}

void initDsAPI(py::module& m)
{
  initArray2dAPI<int>(m, "int");
  initArray2dAPI<double>(m, "float");
  initArray3dAPI<int>(m, "int");
  initArray3dAPI<double>(m, "float");
}

void initRlAPI(py::module& m)
{
  py::class_<RlUtils>(m, "rl_utils")
    .def(py::init<>())
    .def("gen_feats_edges_masks", &RlUtils::genFeatsEdgesMasks)
    .def("gen_feats_edges_masks_with_path", &RlUtils::genFeatsEdgesMasksWithPathInfo)
    .def("gen_feats_edges_masks_with_super_path", &RlUtils::genFeatsEdgesMasksWithSuperPathInfo)
    .def("gen_path_feats_edges", &RlUtils::genPathFeatsEdges)
    .def("gen_super_path_feats_edges", &RlUtils::genSuperPathFeatsEdgesImpl)
    .def("get_wl_via", &RlUtils::getWlVia)
    .def("get_wl_via_drv", &RlUtils::getWlViaDrv)
    .def("add_cur_sol_to_history", &RlUtils::addCurSol2History)
    .def("to_vis_gds", py::overload_cast<const CirDB&, const Net&, const String&>(&RlUtils::toVisGds, py::const_))
    .def("to_vis_gds", py::overload_cast<const CirDB&, const Vector<Int>&, const String&>(&RlUtils::toVisGds, py::const_))
    .def("get_min_wl_via", &RlUtils::getMinWlVia)
    .def("get_layers_wl_via", &RlUtils::getLayersWlVia)
    .def("get_layers_wl_via_drv", &RlUtils::getLayersWlViaDrv)
    .def("get_path_res", &RlUtils::getPathRes)
    .def("get_surrounding_image", &RlUtils::getSurroundingImage);
}

} // namespace PROJECT_NAMESPACE::api

//////////////////////////////////
//          PYBIND11            //
//////////////////////////////////
//PYBIND11_MAKE_OPAQUE(PROJECT_NAMESPACE::FlatHashSet<int>);

PYBIND11_MODULE(autocraft, m)
{
  srand(841125);
  py::bind_vector<Vector<int>>(m, "VectorInt", pybind11::module_local(false));
  py::bind_vector<Vector<Vector<int>>>(m, "VectorVectorInt");
  py::bind_vector<Vector<Vector<Vector<int>>>>(m, "VectorVectorVectorInt");
  py::bind_vector<Vector<Vector<Vector<Vector<int>>>>>(m, "VectorVectorVectorVectorInt");

  py::bind_vector<Vector<double>>(m, "VectorFloat", pybind11::module_local(false));
  py::bind_vector<Vector<Vector<double>>>(m, "VectorVectorFloat");
  py::bind_vector<Vector<Vector<Vector<double>>>>(m, "VectorVectorVectorFloat");
  py::bind_vector<Vector<Vector<Vector<Vector<double>>>>>(m, "VectorVectorVectorVectorFloat");

  py::bind_vector<Vector<Point<int>>>(m, "VectorPointInt");
  py::bind_vector<Vector<Point<double>>>(m, "VectorPointFloat");
  py::bind_vector<Vector<Vector<Point<int>>>>(m, "VectorVectorPointInt");
  py::bind_vector<Vector<Vector<Point<double>>>>(m, "VectorVectorPointFloat");

  py::bind_vector<Vector<Point3d<int>>>(m, "VectorPoint3dInt");
  py::bind_vector<Vector<Point3d<double>>>(m, "VectorPoint3dFloat");
  py::bind_vector<Vector<Vector<Point3d<int>>>>(m, "VectorVectorPoint3dInt");
  py::bind_vector<Vector<Vector<Point3d<double>>>>(m, "VectorVectorPoint3dFloat");

  py::bind_vector<Vector<Segment<int>>>(m, "VectorSegmentInt");
  py::bind_vector<Vector<Segment<double>>>(m, "VectorSegmentFloat");
  py::bind_vector<Vector<Vector<Segment<int>>>>(m, "VectorVectorSegmentInt");
  py::bind_vector<Vector<Vector<Segment<double>>>>(m, "VectorVectorSegmentFloat");

  py::bind_vector<Vector<Segment3d<int>>>(m, "VectorSegment3dInt");
  py::bind_vector<Vector<Segment3d<double>>>(m, "VectorSegment3dFloat");
  py::bind_vector<Vector<Vector<Segment3d<int>>>>(m, "VectorVectorSegment3dInt");
  py::bind_vector<Vector<Vector<Segment3d<double>>>>(m, "VectorVectorSegment3dFloat");

  py::bind_vector<Vector<Box<int>>>(m, "VectorBoxInt");
  py::bind_vector<Vector<Box<double>>>(m, "VectorBoxFloat");
  py::bind_vector<Vector<Vector<Box<int>>>>(m, "VectorVectorBoxInt");
  py::bind_vector<Vector<Vector<Box<double>>>>(m, "VectorVectorBoxFloat");

  py::bind_vector<Vector<Pair<Box<int>, int>>>(m, "VectorPairBoxIntInt");
  py::bind_vector<Vector<Vector<Pair<Box<int>, int>>>>(m, "VectorVectorPairBoxIntInt");

  py::bind_vector<Vector<TopoTree::SegData>>(m, "VectorSegData");
  py::bind_vector<Vector<Vector<TopoTree::SegData>>>(m, "VectorVectorSegData");

  py::bind_vector<Vector<Array2d<int>>>(m, "VectorArray2dInt");
  py::bind_vector<Vector<Array2d<double>>>(m, "VectorArray2dFloat");
  py::bind_vector<Vector<Array3d<int>>>(m, "VectorArray3dInt");
  py::bind_vector<Vector<Array3d<double>>>(m, "VectorArray3dFloat");

  //spdlog::set_level(spdlog::level::err);
  spdlog::set_pattern("[%^%l%$] %v");
  PROJECT_NAMESPACE::api::initDbAPI(m);
  PROJECT_NAMESPACE::api::initParserAPI(m);
  PROJECT_NAMESPACE::api::initPlacerAPI(m);
  PROJECT_NAMESPACE::api::initRouterAPI(m);
  PROJECT_NAMESPACE::api::initDrcMgrAPI(m);
  PROJECT_NAMESPACE::api::initWriterAPI(m);
  PROJECT_NAMESPACE::api::initGeoAPI(m);
  PROJECT_NAMESPACE::api::initDsAPI(m);
  PROJECT_NAMESPACE::api::initRlAPI(m);
} 

