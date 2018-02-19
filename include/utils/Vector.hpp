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

#include <array>
#include <iterator>
#include <cmath>
#include <type_traits>

namespace sat {
namespace utils {
	
template<typename T, int dim>
class Vector;

template<typename T, int dim>
typename Vector<T, dim>::container::iterator begin(Vector<T, dim>& vector);

template<typename T, int dim>
typename Vector<T, dim>::container::const_iterator begin(const Vector<T, dim>& vector);

template<typename T, int dim>
typename Vector<T, dim>::container::iterator end(Vector<T, dim>& vector);

template<typename T, int dim>
typename Vector<T, dim>::container::const_iterator end(const Vector<T, dim>& vector);

//template<typename T, int dim>
//std::ostream& operator<<(std::ostream& stream, const Vector<T, dim>& vector);


// TODO: add constexpr where possible
/**
 *	@brief This templated class represents an N-dimensional vector.
 *	
 *	It behaves as an std::array and supports some element-wise operations.
 *	It can be used for example to represent the angular position of
 *	the nano satellite in the space (3D state) or a 10 DOF IMU reading
 *	with angular speed, acceleration, magnetic field and pressure (10D state).
 *	
 *	@param T: datatype to be used (int, float, double... or even another Vector<>)
 *	@param dim: number of size of the state
 */
template<typename T, int dim>
class Vector {

public:

	using value_type = T;
	static const std::size_t size = dim;

	using container = std::array<value_type, dim>;


	// TODO: improve constructors!

	Vector() {
		fill(value_type());
	}

	template <typename... Type>
	constexpr Vector(Type... ts) : values{ ts... } { }

	constexpr Vector(const container &init) : values(init) { }

	virtual ~Vector() = default;


	constexpr value_type& operator[](std::size_t idx) {
		return values[idx];
	}

	constexpr const value_type& operator[](std::size_t idx) const {
		return values[idx];
	}

	Vector<value_type, size>& operator+=(const Vector<value_type, size>& lhs) {
		for (size_t i = 0; i < size; ++i)
			values[i] += lhs[i];

		return *this;
	}

	Vector<value_type, size>& operator-=(const Vector<value_type, size>& lhs) {
		for (size_t i = 0; i < size; ++i)
			values[i] -= lhs[i];

		return *this;
	}

	// TODO: an enable_if could be useful here? Normalization should work
	//		 only if value_type is an arithmetic
	constexpr Vector<value_type, size>& normalize() {
		T rootSquaredSum = 0;
		for (auto val : values)
			rootSquaredSum += val*val;

		rootSquaredSum = sqrt(rootSquaredSum);
		for (auto& val : values)
			val /= rootSquaredSum;

		return *this;
	}

	void fill(const value_type &value) {
		values.fill(value);
	}

	friend typename container::iterator begin<T, dim>(Vector<T, dim>& vector);
	friend typename container::const_iterator begin<T, dim>(const Vector<T, dim>& vector);

	friend typename container::iterator end<T, dim>(Vector<T, dim>& vector);
	friend typename container::const_iterator end<T, dim>(const Vector<T, dim>& vector);

	//friend std::ostream& operator<< <T,dim>(std::ostream& stream, const Vector<T, dim>& vector);

protected:
	container values;

};


template<typename T, int dim>
typename Vector<T, dim>::container::iterator begin(Vector<T, dim>& vector) {
	using std::begin;
	return begin(vector.values);
}

template<typename T, int dim>
typename Vector<T, dim>::container::const_iterator begin(const Vector<T, dim>& vector) {
	using std::begin;
	return begin(vector.values);
}

template<typename T, int dim>
typename Vector<T, dim>::container::iterator end(Vector<T, dim>& vector) {
	using std::end;
	return end(vector.values);
}

template<typename T, int dim>
typename Vector<T, dim>::container::const_iterator end(const Vector<T, dim>& vector) {
	using std::end;
	return end(vector.values);
}

template<typename T, int dim>
std::ostream& operator<<(std::ostream& stream, const Vector<T, dim>& vector) {
	stream << "(";
	for (auto i = 0; i < dim-1; ++i)
		stream << vector[i] << ",";
	stream << vector[dim-1] << ")";
	return stream;
}

template<typename T, int dim>
Vector<T, dim> operator+(Vector<T, dim> lhs, const Vector<T, dim>& rhs) {
	lhs += rhs;
	return lhs;
}

template<typename T, int dim>
Vector<T, dim> operator-(Vector<T, dim> lhs, const Vector<T, dim>& rhs) {
	lhs -= rhs;
	return lhs;
}

template<typename T, int dim>
constexpr bool operator==(const Vector<T, dim>& lhs, const Vector<T, dim>& rhs) {
	for (auto i = 0; i < dim; ++i)
		if (lhs[i] != rhs[i])
			return false;
	return true;
}

template<typename T, int dim>
constexpr bool operator!=(const Vector<T, dim>& lhs, const Vector<T, dim>& rhs) {
	return !(lhs == rhs);
}

template<typename T, int dim>
Vector<T, dim> operator*(float coeff, const Vector<T, dim>& s) {
	Vector<T, dim> resul;
	for (auto i = 0; i < dim; ++i) resul[i] = coeff*s[i];
	return resul;
}

template<typename T, int dim>
Vector<T, dim> operator*(const Vector<T, dim>& s, float coeff) {
	return coeff*s;
}

template<typename T, int dim>
Vector<T, dim> operator/(const Vector<T, dim>& s, float coeff) {
	Vector<T, dim> resul;
	for (auto i = 0; i < dim; ++i) resul[i] = s[i] / coeff;
	return resul;
}


/**
 * @brief multiplies two arithmetic scalars
 */
template<typename T, typename = std::enable_if_t<std::is_arithmetic<T>::value>>
T multElementwise(T v1, T v2) {
   return v1 * v2;
}

/**
 * @brief multiplies two utils::Vector (or utils::Matrix) element wise
 */
template<typename T, int dim>
Vector<T, dim> multElementwise(const Vector<T, dim>& v1, const Vector<T, dim>& v2) {
   Vector<T, dim> result;
   for (size_t i = 0; i < dim; ++i) 
      result[i] = multElementwise(v1[i],v2[i]);
   return result;
}

/**
 * @brief divides two arithmetic scalars
 */
template<typename T, typename = std::enable_if_t<std::is_arithmetic<T>::value>>
T divElementwise(T v1, T v2) {
   return v1 / v2;
}

/**
 * @brief divides two utils::Vector (or utils::Matrix) element wise
 */
template<typename T, int dim>
Vector<T, dim> divElementwise(const Vector<T, dim>& v1, const Vector<T, dim>& v2) {
   Vector<T, dim> result;
   for (size_t i = 0; i < dim; ++i) 
      result[i] = divElementwise(v1[i],v2[i]);
   return result;
}

/**
 *	@brief Extension of std::isnan to Vector. Calls isnan on
 *			each element of Vector.
 *	@return true if at least one element of vec isnan, false otherwise
 */
template<typename T, int dim>
bool isnan(const sat::utils::Vector<T, dim> &vec) {

	using std::isnan;

	for (const auto &value : vec) {
		if (isnan(value)) return true;
	}
	return false;
}

/**
 *	@brief Extension of std::isnormal to Vector. Calls isnormal on
 *			each element of Vector.
 *	@return true if all elements of vec isnormal, false otherwise
 */
template<typename T, int dim>
bool isnormal(const sat::utils::Vector<T, dim> &vec) {

	using std::isnormal;

	for (const auto &value : vec) {
		if (!isnormal(value)) return false;
	}
	return true;
}

/**
 *	@brief Extension of std::abs to Vector. Calls abs on
 *			 each element of Vector.
 *	@return a positive Vector
 */
template<typename T, int dim>
Vector<T,dim> abs(const Vector<T, dim> &vec) {
   using std::abs;
   
   Vector<T, dim> output;
   for (auto i = 0; i < dim; ++i) {
      output[i] = abs(vec[i]);
   }

   return output;
}

} // namespace utils
} // namespace sat