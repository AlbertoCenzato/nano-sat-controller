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

#include <string>
#include <vector>
#include <chrono>
#include <thread>

#include "devices/Interfaces.hpp"
#include "utils/DataTypes.hpp"

namespace sat {

class NanoSat;
namespace device {
	class IMU10DOF;
	class ISensor; 
}
namespace utils  { class UI; }


namespace test {

class Test {

public:

	explicit Test(const NanoSat* satellite);

	void currentSensor() const;
	void dcMotor() const;
	void testLogger() const;
	void imu10DOF() const;
	void accelerometer() const;
	void gyroscopeAngles() const;
	void gyroscopeOffsets() const;
	void magnetometer() const;
	void coil() const;
	void feedbackFreq() const;
	void camera() const;

	void availability() const;
	void readSpeed() const;
	void motorAxes() const;
   void testGyroscopeImuError() const;
   void testIMUTracking() const;
   void powerBoard() const;

private:

	static const std::string brk;		// break line, a string of "--------"

	const NanoSat* satellite_;

	std::string header(const std::string& str) const;

	void readLoop(const device::ISensor* sensor, const std::string& header) const;
	void readLoop(const std::vector<const device::ISensor*>& sensors, const std::string& header) const;

	utils::Vector3f getAngle(device::IMU10DOF* imu) const;

};


template<typename Rep, typename Period>
std::vector<double> readMeanFor(const device::ISensor *sensor,
   const std::chrono::duration<Rep, Period> &duration) {

   const auto start = std::chrono::high_resolution_clock::now();
   auto now = std::chrono::high_resolution_clock::now();

   long count = 0;
   auto sum = sensor->read();
   while (now < start + duration) {
      auto value = sensor->read();
      ++count;

      for (auto i = 0; i < sum.size(); ++i) {
         sum[i] += value[i];
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      now = std::chrono::high_resolution_clock::now();
   }

   for (auto &val : sum) {
      val /= count;
   }

   return sum;
}


} // namespace test
} // namespace sat