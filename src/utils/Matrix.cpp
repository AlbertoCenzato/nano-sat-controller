/*====================================================================
Nano satellite controller

// Copyright: Copyright (c) 2017, Alberto Cenzato
All rights reserved.

// Licence: GNU

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU License.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU License V3 for more details: https://www.gnu.org/licenses/gpl-3.0.html

//==================================================================== */

#include "utils/Matrix.hpp"

namespace sat {
namespace utils {


RotationMat::RotationMat() {
	for (auto i = 0; i < size; ++i) {
		values[i][i] = 1;
	}
}

RotationMat::RotationMat(float x00, float x01, float x02, float x10, float x11, float x12, float x20, float x21, float x22) {
	values[0] = { x00, x01, x02 };
	values[1] = { x10, x11, x12 };
	values[2] = { x20, x21, x22 };
}


RotationMat RotationMat::t() {
return RotationMat(values[0][0], values[1][0], values[2][0],
				   values[0][1], values[1][1], values[2][1],
				   values[0][2], values[1][2], values[2][2]);
}


RotationMat RotationMat::zero() {
	RotationMat R;
	for (auto i = 0; i < size; ++i) {
		R[i][i] = 0;
	}
	return R;
}

RotationMat RotationMat::I() {
	return RotationMat();
}


Vector<float, 3> operator*(const RotationMat& R, const Vector<float, 3>& values) {
   Vector<float, 3> out;
   for (auto i = 0; i < RotationMat::size; ++i) {
      out[i] = 0;
      for (auto j = 0; j < RotationMat::size; ++j) {
         out[i] += R[i][j] * values[j];
      }
   }
   return out;
}

} // namespace utils
} // namespace sat
