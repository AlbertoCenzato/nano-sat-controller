// /*===========================================================================
//                              NanoSatController
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

#pragma once

#include "utils/DataTypes.hpp"

namespace sat {

   namespace device { class IMU10DOF; }

   class NanoSat;

namespace test {
   
class Calibration {
   
   NanoSat* satellite_;

   utils::Vector3f getAngle(device::IMU10DOF* imu) const;

public:
   explicit Calibration(NanoSat* satellite);

   utils::Vector3f calibrateGyroscopeGains() const;
   utils::Vector3f calibrateMagnetometerOffsets() const;
};

} // namespace sat
} // namespace test