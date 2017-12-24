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

#pragma once

#include <utils/Vector.hpp>

namespace sat {
namespace utils {

template<typename T, int n, int m>
using Matrix = Vector<Vector<T, m>, n>;
using Matrix3f3 = Matrix<float, 3, 3>;

// TODO: add constexpr where possible
class RotationMat : public Matrix3f3 {

public:
	RotationMat();
	RotationMat(float x00, float x01, float x02,
				float x10, float x11, float x12,
				float x20, float x21, float x22);

	/**
	 *	@brief Returns the transpose (which is also the inverse) of the rotation mat
	 */
	RotationMat t();

	/**
	 *	@brief Gives an all 0 matrix
	 */
	static RotationMat zero();

	/**
	 *	@brief Gives the identity matrix
	 */
	static RotationMat I();

};


Vector<float, 3> operator*(const RotationMat& R, const Vector<float, 3>& values);

} // namespace utils
} // namespace sat