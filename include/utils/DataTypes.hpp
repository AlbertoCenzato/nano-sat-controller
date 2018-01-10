/*====================================================================
Nano satellite controller

// Copyright   : Copyright (c) 2017, Alberto Cenzato
All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU License.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU License V2 for more details: https://www.gnu.org/licenses/gpl-2.0.html

//==================================================================== */


#pragma once

/**------------------------------------------------------------------------
 * This is a unique include file for all project specific data types
 * ------------------------------------------------------------------------*/

#include "utils/Vector.hpp"
#include "utils/Matrix.hpp"
#include "spdlog/spdlog.h"

namespace sat {
namespace utils{


enum class Axis {
	NONE = -1,
	X,
	Y,
	Z,
	ALL
};


inline Axis operator*(const RotationMat& rotMat, Axis axis) {
   if (axis == Axis::NONE)
      return Axis::NONE;

   if (axis == Axis::ALL)
      return Axis::ALL;

   auto index = int(axis);
   for (auto i = 0; i < rotMat.size; ++i) {
      if (rotMat[i][index] != 0)
         return Axis(i);
   }

   return Axis::NONE;
}


using Vector3i  = Vector<int, 3>;
using Vector3f  = Vector<float, 3>;
using Vector3d  = Vector<double, 3>;
using Vector4f  = Vector<float, 4>;
using Vector10f = Vector<float, 10>;


using LogLevel = spdlog::level::level_enum;

// portable way to have greek pi as a constant
// PI is not in the standard so if a PI constant
// is defined or not is implementation-dependent
const double PI = 3.1415926535897932384626433832795;

} // namespace utils

using utils::PI;

} // namespace sat