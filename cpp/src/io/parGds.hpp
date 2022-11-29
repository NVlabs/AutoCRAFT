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

#include <limbo/parsers/gdsii/gdsdb/GdsIO.h>
#include <limbo/parsers/gdsii/gdsdb/GdsObjectHelpers.h>

#include "global/global.hpp"
#include "db/dbCir.hpp"

PROJECT_NAMESPACE_START

struct PolygonLayer {
	explicit PolygonLayer() = default;
	String toStr() const {
		std::stringstream oss;
		oss << "Layer " << layer << " ";
		for (const auto& pt : pts) {
			oss << " "<< pt << " ";
		}
		return oss.str();
	}
	/// @brief scale the polygon
	void scale(const Real scaleX, const Real scaleY) {
		for (size_t ptIdx = 0; ptIdx < pts.size(); ++ptIdx) {
			const Real x = pts.at(ptIdx).x();
			const Real y = pts.at(ptIdx).y();
			const Int newX = std::round(x * scaleX);
			const Int newY = std::round(y * scaleY);
			pts.at(ptIdx).setX(newX);
			pts.at(ptIdx).setY(newY);
		}
	}

	///Members
	Vector<Point<Int>> pts; ///< The points of the polygon
	Int layer = MAX_INT; ///< The layer of the polygon
};

class GdsReader { // from limbo
 public:
	GdsReader(CirDB& c)
		: _cir(c) {}
	~GdsReader() {}

	void parseRouting(const String& filename, const Vector<Int>& vNetIds = Vector<Int>());

 private:
	CirDB&                _cir;
  FlatHashMap<Int, Int> _maskIdx2LayerIdx;
	Vector<PolygonLayer>  _vPolygonLayers;
	/////////////////////////////////////////
	//    Private functions                //
	/////////////////////////////////////////
	String  topCell(const GdsParser::GdsDB::GdsDB& db);
	void    buildLayerMap();
  void    saveShapes(const Vector<Int>& vNetIds);

};
//////////////////////////////////////////////////////////////////
/// @brief detailed struct for the functions of processing gds shapes
namespace ExtractShapeLayerActionDetails {

	namespace gtl = boost::polygon;
	/// @brief convert from gds layer to router layer
	/// @return MAX_INT if not found, otherwise, the router layer
	inline Int gdsLayer2LayerIdx(const Int gdsLayer, const FlatHashMap<Int,Int>& map) {
		/// Linear search in the map
    if (map.find(gdsLayer) != map.end()) {
      return map.at(gdsLayer);
    }
		return MAX_INT;
	}
	/// @brief default action
	template<typename ObjectType>
  inline void extractShape(FlatHashMap<Int, Int>& map, Vector<PolygonLayer>& polygons, ::GdsParser::GdsRecords::EnumType type, ObjectType* object) {}


	/// @brief process gds rectangle
	template<>
  inline void extractShape(FlatHashMap<Int, Int>& map, Vector<PolygonLayer>& polygons, ::GdsParser::GdsRecords::EnumType type, ::GdsParser::GdsDB::GdsRectangle* object) {
    //DBG("Rect \n");
    assert(false);
    //AssertMsg(0, "%s: Rectangle type not support yet \n", __FUNCTION__);
    //DBG("Gds rectangle layer %d, (%d, %d ), (%d, %d) \n", object->layer(), gtl::xl(object), gtl::yl(object), gtl::xh(object), gtl::yh(object));
  }

	/// @brief process gds polygon
	template<>
  inline void extractShape(FlatHashMap<Int, Int>& map, Vector<PolygonLayer>& polygons, ::GdsParser::GdsRecords::EnumType type, ::GdsParser::GdsDB::GdsPolygon* object) {
    /// Convert the gds layer to the router layer
    const Int layerIdx = gdsLayer2LayerIdx(object->layer(), map);
    if (layerIdx != MAX_INT) {
      polygons.emplace_back(PolygonLayer());
      polygons.back().layer = layerIdx;
      /// Add points
      assert(object->size() == 5);
      for (auto pt : *object) {
        polygons.back().pts.emplace_back(Point<Int>(pt.x(), pt.y()));
      }
    }
  }

	/// @brief process path
	template<>
  inline void extractShape(FlatHashMap<Int, Int>& map, Vector<PolygonLayer>& polygons, ::GdsParser::GdsRecords::EnumType type, ::GdsParser::GdsDB::GdsPath* object) {
    const Int layerIdx = gdsLayer2LayerIdx(object->layer(), map);
    if (layerIdx != MAX_INT) {
      polygons.emplace_back(PolygonLayer());
      polygons.back().layer = layerIdx;

      assert(object->size() == 2);
      Int mnx = MAX_INT, mny = MAX_INT;
      Int mxx = MIN_INT, mxy = MIN_INT;
      for (auto pt : *object) {
        mnx = std::min(pt.x(), mnx);
        mxx = std::max(pt.x(), mxx);
        mny = std::min(pt.y(), mny);
        mxy = std::max(pt.y(), mxy);
      }
      assert(object->pathtype() == std::numeric_limits<int>::max() or // default pathtype in Limbo
             object->pathtype() == 0 or // 0 should be default (no eol extension)
             object->pathtype() == 1 or
             object->pathtype() == 2 or 
             object->pathtype() == 4);
      const Int ext = object->width() / 2;
      if (mnx == mxx) { // vertical
        mnx -= ext;
        mxx += ext;
        if (object->pathtype() == 1) {
          // round end (shouldn't happen)
          assert(false);
        }
        else if (object->pathtype() == 2) {
          mny -= ext;
          mxy += ext;
        }
        else if (object->pathtype() == 4) {
          // custom square end (not supported by limbo currently)
          assert(false);
        }
      }
      else if (mny == mxy) { // horizontal
        mny -= ext;
        mxy += ext;
        if (object->pathtype() == 1) {
          // round end (shouldn't happen)
          assert(false);
        }
        else if (object->pathtype() == 2) {
          mnx -= ext;
          mxx += ext;
        }
        else if (object->pathtype() == 4) {
          // custom square end (not supported by limbo currently)
          assert(false);
        }
      }
      else {
        assert(false);
      }
      polygons.back().pts.emplace_back(Point<Int>(mnx, mny));
      polygons.back().pts.emplace_back(Point<Int>(mxx, mny));
      polygons.back().pts.emplace_back(Point<Int>(mxx, mxy));
      polygons.back().pts.emplace_back(Point<Int>(mnx, mxy));
      polygons.back().pts.emplace_back(Point<Int>(mnx, mny));
    }
  }

}


/// @brief aution function object to process the the 
struct ExtractShapeLayerAction {
	/// @param first: the map between the gds layer indices to the router layers. Only care about the mapping within this map
	/// @param second: a reference to a vector of polygons, saved the read polygons in this vector
	ExtractShapeLayerAction(FlatHashMap<Int, Int>& map, Vector<PolygonLayer>& polygons) : _maskIdx2LayerIdx(map), _polygons(polygons) {}
	template<typename ObjectType>
  void operator()(::GdsParser::GdsRecords::EnumType type, ObjectType* object) {
    bool b = type == ::GdsParser::GdsRecords::BOUNDARY or
             type == ::GdsParser::GdsRecords::PATH or 
             type == ::GdsParser::GdsRecords::TEXT;
    if (!b) {
      std::cerr << type << std::endl;
    }
    assert(b);
    ExtractShapeLayerActionDetails::extractShape(_maskIdx2LayerIdx, _polygons, type, object);
  }

	/// @return a message of action for debug
	String message() const {
		return "ExtractShapeLayerAction";
	}

	FlatHashMap<Int, Int>& _maskIdx2LayerIdx; ///< Map gds layer id to all layers in router (masterslice + routing + cut)
	Vector<PolygonLayer>&  _polygons; ///< The polygons read from the gds
};


namespace GetSRefNameActionDetailsParser {
	/// @brief default type
	template<typename ObjectType>
  inline void getSName(String &name,  ::GdsParser::GdsRecords::EnumType type, ObjectType *object) {}

	/// @brief SREF type
	template<>
  inline void getSName(String &name,  ::GdsParser::GdsRecords::EnumType type, ::GdsParser::GdsDB::GdsCellReference *object) {
    name = object->refCell();
  }
}

/// @brief aution function object to get the cell reference name
struct GetSRefNameActionParser {
	/// @param A reference to the string to record the name of the sref
	GetSRefNameActionParser(String& name) : _name(name) {}
	template<typename ObjectType>
  void operator()(::GdsParser::GdsRecords::EnumType type, ObjectType* object) {
    GetSRefNameActionDetailsParser::getSName(_name, type, object);
  }

	/// @return a message of action for debug
	String message() const  {
		return "GetSRefNameAction";
	}


	String& _name; ///< The cell reference name
};


PROJECT_NAMESPACE_END

