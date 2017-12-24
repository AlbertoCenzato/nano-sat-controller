// /*===========================================================================
//                                                                      NanoSatController
// 
// // Copyright   : Copyright (c) 2017, Alberto Cenzato
//        All rights reserved.
// 
// // Licence: GNU
// 
// // Based on: 
// 
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU License.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU License V3 for more details: https://www.gnu.org/licenses/gpl-3.0.html
// 
// //============================================================================ */

#include "test/Calibration.hpp"

#include "NanoSat.hpp"
#include "devices/IMU10DOF.hpp"
#include "utils/UI.hpp"

using namespace sat::utils;

namespace sat {
namespace test {
   
   Calibration::Calibration(NanoSat * satellite) : satellite_(satellite) { }


   Vector3f Calibration::calibrateGyroscopeGains() const {

   //string hdr = header("Gyroscope calibration procedure");

   auto imu = satellite_->getIMU();
   if (!imu->getGyroscope()->isAvailable()) {
      Log::info << "Gyroscope unavailable";
      Vector3f vec;
      vec.fill(std::numeric_limits<float>::quiet_NaN());
      return vec;
   }

   const float X_FINE_CORSA = 65.0858f;
   const float Y_FINE_CORSA = 42.2737f;

   Vector3f gains;

   cout << "Portare il nano satellite a fine corsa.\n" <<
      "Poi ruotare lentamente il nano satellite sull'asse X fino a portarlo a fine corsa sul lato opposto." << std::endl;
   ui::waitKey();

   auto angles = getAngle(imu);
   gains[0] = 2 * X_FINE_CORSA / angles[0];

   cout << "Riportare il nano satellite in posizione iniziale" << std::endl;
   cout << "Portare il nano satellite in posizione orizzontale.\n" <<
      "Poi ruotare lentamente il nano satellite sull'asse Y fino a portarlo a fine corsa." << std::endl;
   ui::waitKey();

   angles = getAngle(imu);
   gains[1] = Y_FINE_CORSA / angles[1];

   cout << "Riportare il nano satellite in posizione iniziale" << std::endl;
   cout << "Ruotare lentamente il nano satellite sull'asse Z di 720 gradi" << std::endl;
   ui::waitKey();

   angles = getAngle(imu);
   gains[2] = 720.f / angles[2];

   cout << "Gains: " << gains << endl;

   return gains;
}


Vector3f Calibration::calibrateMagnetometerOffsets() const {

   const auto imu = satellite_->getIMU();
   if (!imu->getMagnetometer()->isAvailable()) {
      Log::info << "Magnetometer unavailable";
   }

   Vector3f max, min;
   max.fill(std::numeric_limits<float>::min());
   min.fill(std::numeric_limits<float>::max());

   const auto start = chrono::high_resolution_clock::now();
   auto currTime = chrono::high_resolution_clock::now();
   while (currTime < start + chrono::seconds(90)) {
      auto values = imu->readMagnetometerRaw();

      for (auto i = 0; i < 3; ++i) {
         if (values[i] > max[i]) max[i] = values[i];
         if (values[i] < min[i]) min[i] = values[i];
      }

      ui::clearConsole();
      cout << "(X, Y, Z): " << values << endl;

      this_thread::sleep_for(chrono::milliseconds(50));
      currTime = chrono::high_resolution_clock::now();
   }

   auto offsets = (max + min) / 2;
   std::cout << "Offsets (X,Y,Z): " << offsets << std::endl;

   return offsets;
}


Vector3f Calibration::getAngle(device::IMU10DOF* imu) const {
   const auto start = chrono::high_resolution_clock::now();
   Vector3f angles{ 0.f, 0.f, 0.f };
   auto prevTime = chrono::high_resolution_clock::now();
   auto currTime = chrono::high_resolution_clock::now();
   //imu->startReading();
   auto prevValues = imu->readGyrosope();
   long count = 0;
   while (currTime < start + chrono::seconds(20)) {

      ++count;

      currTime = chrono::high_resolution_clock::now();
      auto currValues = imu->readGyrosope();

      float dt = chrono::duration_cast<chrono::microseconds>(currTime - prevTime).count() / 10E6f;
      auto area = (prevValues + currValues)*(dt / 2);
      angles += area;

      ui::clearConsole();
      cout << "Read values: " << currValues << endl;
      cout << "Angles:      " << angles << endl;

      prevTime = currTime;
      prevValues = currValues;
      this_thread::sleep_for(chrono::milliseconds(5));
   }

   cout << "Misure effettuate: " << count << endl;

   return angles;
}

} // namespace test
} // namespace sat
