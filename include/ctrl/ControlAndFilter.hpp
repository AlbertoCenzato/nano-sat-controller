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

#include <chrono>
#include <memory>

#include "utils/DataTypes.hpp"

/* ------------------------------------------------------------------
 * - This file contains all the control and filter functions needed -
 * - by NanoSat, Controller and IMU                                 -
 * ------------------------------------------------------------------*/

namespace sat {
namespace ctrl {

/**
 *	@brief Generic prototype of a control function. Its only public member is
 *		    the function call operator which receives current state and target
 *		    state and gives as output the necessary control feedback.
 */
template<typename T>
struct ControlAlgorithm {
	
	virtual ~ControlAlgorithm() = default;
	virtual T operator()(T state, T target) = 0;

};

/**
 *	@brief PID is an example of a specific control function
 *		    implementing a proportional, integral, derivative controller.
 *	It receives as argument two templated objects of type T for which
 *	all arithmetic operations with T as well as with
 *	scalars should be defined (for example any numeric base type as int
 *	or float can be used or, if a vector is needed, std::utils::Vector<> can be used.
 */
template<typename T>
struct PID : ControlAlgorithm<T> {

	float kp, ki, kd; // proportional, integral and derivative gains
	std::chrono::time_point<std::chrono::system_clock> lastT;
	T lastError;
	T integral;

	explicit PID(float kp = 0, float ki = 0, float kd = 0) : kp(kp), ki(ki), kd(kd) { }

	T operator()(T state, T target) override {
      auto error = state - target;
		auto now = std::chrono::high_resolution_clock::now();
		auto dt = std::chrono::duration_cast<std::chrono::nanoseconds>(now - lastT).count();
		integral += (lastError + error)*dt / 2;	// numerically integrating using trapezioidal rule
		auto p = kp*error;
		auto i = ki*integral;
		auto d = kd*(error - lastError);
		lastError = error;
		lastT = now;

		return p + i + d;
	}

	static std::unique_ptr<PID<T>> create(float kp = 0, float ki = 0, float kd = 0) {
		return std::make_unique<PID<T>>(kp, ki, kd);
	}
};

/**
 *	@brief Generic prototype of a filter function. Its only public member is
 *		    the function call operator which receives a value and gives as output 
 *		    the the filtered value.
 */
template<typename In, typename Out>
struct FilterAlgorithm {

	virtual ~FilterAlgorithm() = default;
	virtual Out operator()(In input) = 0;
   virtual void reset() = 0;
   virtual void resetClock() = 0;

};

using IMUSensorFusionAlg = FilterAlgorithm<utils::Matrix<float,3,3>, utils::Vector4f>;


/**
* @brief Quaternion implementation of the 'DCM filter' [Mahoney et al].
* 
* Incorporates the magnetic distortion compensation algorithms
* from Sebastian Madgwick filter which eliminates the need for a reference
* direction of flux (bx bz) to be predefined and limits the effect
* of magnetic distortions to yaw axis only.
* 
* @note see: http://x-io.co.uk/open-source-ahrs-with-x-imu/
*		  and also: http://ieeexplore.ieee.org/document/4608934/
*/
struct MahoneyFilter :  IMUSensorFusionAlg {
	
private:	
   const float twoKp;  // 2 * proportional gain (Kp)
   const float twoKi;  // 2 * integral gain (Ki)
	utils::Vector4f q;  // quaternion of sensor frame relative to auxiliary frame
	utils::Vector3f integralFB;

	// sample period clocks
	std::chrono::time_point<std::chrono::high_resolution_clock> lastUpdate, now;


public:
	explicit MahoneyFilter(float kp = 0.5f, float ki = 0.1f);

	utils::Vector4f operator()(utils::Matrix<float,3,3> vec) override;

	void reset() override;
   void resetClock() override;

	static std::unique_ptr<MahoneyFilter> create(float kp = 0.5f, float ki = 0.1f);

};



struct MadgwickFilter : IMUSensorFusionAlg {

private:
	float samplePeriod;
	const float beta;
	utils::Vector4f q; // quaternion of sensor frame relative to auxiliary frame

	// sample period expressed in milliseconds
	std::chrono::time_point<std::chrono::high_resolution_clock> lastUpdate, now;


public:

	explicit MadgwickFilter(float beta = 0.1f);

	utils::Vector4f operator()(utils::Matrix<float, 3, 3> vec) override;
	void Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	void Update(float gx, float gy, float gz, float ax, float ay, float az);

	void reset() override;
   void resetClock() override;

	static std::unique_ptr<MadgwickFilter> create();

};

} // namespace ctrl
} // namespace sat