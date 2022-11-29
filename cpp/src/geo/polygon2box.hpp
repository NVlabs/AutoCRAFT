/* 
 * Copyright Hao Chen 2020 - 2022.
 * Distributed under the Boost Software License, Version 1.0.
 * (See accompanying file LICENSE_1_0.txt or copy at
 *  https://www.boost.org/LICENSE_1_0.txt)
 */

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

#include "box.hpp"
#include <limbo/geometry/Polygon2Rectangle.h>

namespace limbo::geometry {
	////////////////////////////////////////
	//   point_traits                     //
	////////////////////////////////////////
	template<typename T>
	struct point_traits<PROJECT_NAMESPACE::Point<T>> {
		typedef PROJECT_NAMESPACE::Point<T> point_type;
		typedef T coordinate_type;

		static coordinate_type get(const point_type& point, orientation_2d orient)
    {
			if (orient == HORIZONTAL)
				return point.x();
			else if (orient == VERTICAL)
				return point.y();
			else
				assert(false); 
		}

		static void set(point_type& point, orientation_2d orient, coordinate_type value)
    {
			if (orient == HORIZONTAL)
				return point.setX(value);
			else if (orient == VERTICAL)
				return point.setY(value);
			else
				assert(false);
		}

		static point_type construct(coordinate_type x, coordinate_type y)
    {
			return point_type(x, y);
		}
	};
	////////////////////////////////////////
	//   rectangle_traits                 //
	////////////////////////////////////////
	template<typename T>
	struct rectangle_traits<PROJECT_NAMESPACE::Box<T>> {
		typedef PROJECT_NAMESPACE::Box<T> rectangle_type;
		typedef T coordinate_type;
		
		static coordinate_type get(const rectangle_type& rect, direction_2d dir)
    {
			switch (dir) {
				case LEFT:
					return rect.xl();
				case BOTTOM:
					return rect.yl();
				case RIGHT:
					return rect.xh();
				case TOP:
					return rect.yh();
				default:
					assert(false);
			}
		}

		static void set(rectangle_type& rect, direction_2d dir, coordinate_type value)
		{
			switch (dir) {
				case LEFT:
					rect.setXL(value);
					break;
				case BOTTOM:
					rect.setYL(value);
					break;
				case RIGHT:
					rect.setXH(value);
					break;
				case TOP:
					rect.setYH(value);
					break;
				default:
					assert(false);
			}
		}

		static rectangle_type construct(coordinate_type xl, coordinate_type yl, coordinate_type xh, coordinate_type yh)
		{
			return rectangle_type(xl, yl, xh, yh);
		}
	};
}

PROJECT_NAMESPACE_START

namespace geo {
  template<typename T>
  inline bool polygon2Box(const Vector<Point<T>>& vPts, Vector<Box<T>>& vBoxes) {
      limbo::geometry::Polygon2Rectangle<Vector<Point<T>>, Vector<Box<T>>> p2r(vBoxes, vPts.begin(), vPts.end(), limbo::geometry::HOR_VER_SLICING);
      return p2r();
  }
}

PROJECT_NAMESPACE_END

