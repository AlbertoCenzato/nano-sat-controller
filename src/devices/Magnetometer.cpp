/*===========================================================================
Nano satellite controller

// Copyright: Copyright (c) 2017, Alberto Cenzato
All rights reserved.

// Licence: GNU

// Based on: HMC5883L.cpp by Love Electronics (loveelectronics.co.uk)

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU License.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU License V3 for more details: https://www.gnu.org/licenses/gpl-3.0.html

//============================================================================ */

#include "devices/Magnetometer.hpp"

#include <limits>

#include "utils/Settings.hpp"
#include "self_test/SelfTest.hpp"


#pragma region registers
#define ConfigurationRegisterA 0x00
#define ConfigurationRegisterB 0x01
#define ModeRegister				 0x02
#define DataRegisterBegin		 0x03
#define IdentityRegister		 0x0A
#define IdentityRegisterValue  0x48
#pragma endregion registers

#define Measurement_Continuous 0x00
#define Measurement_SingleShot 0x01
#define Measurement_Idle		 0x03

#define ErrorCode_1 "Entered scale was not valid, valid gauss values are: 0.88, 1.3, 1.9, 2.5, 4.0, 4.7, 5.6, 8.1"
#define ErrorCode_1_Num 1

using std::string;
using sat::utils::Vector3i;
using sat::utils::Vector3f;

namespace sat {
namespace device {

const string Magnetometer::DEFAULT_DEV_ID = "HMC5883L_3Axis_Magnetometer";

// ---------- Constructors ----------
#pragma region cunstructors

std::unique_ptr<Magnetometer> Magnetometer::create(const utils::MagnetometerSettings & settings) {
	return std::make_unique<Magnetometer>(settings);
}

Magnetometer::Magnetometer() : DeviceI2C(DEFAULT_DEV_ID, DEFAULT_I2C_ADDR) { }

Magnetometer::Magnetometer(const utils::MagnetometerSettings& settings)	: DeviceI2C(settings.deviceID, settings.address) {
   setOffsets(settings.offsets);
   setGains(settings.gains);
	setMeasurementMode(Measurement_Continuous);
}

#pragma endregion cunstructors


// ---------- Member Functions ----------
#pragma region functions

vector<double> Magnetometer::read() const {
	vector<double> values(3);
	auto val = readMagnetometer();
	values[0] = val[0];
	values[1] = val[1];
	values[2] = val[2];
	return values;
}


Vector3f Magnetometer::readMagnetometer() const {
   auto val = readMagnetometerRaw() - offsets_;
   for (auto i = 0; i < val.size; ++i)
      val[i] *= gains_[i];
   return val;
}

Vector3f Magnetometer::readMagnetometerRaw() const {
	if (!available) {
		const auto nan = numeric_limits<float>::quiet_NaN();
		return {nan, nan, nan};
	}

	uint8_t buffer[6];
	readX(DataRegisterBegin, buffer, 6);
	return Vector3f{float(int16_t((buffer[0] << 8) | buffer[1])),
						 float(int16_t((buffer[2] << 8) | buffer[3])),
						 float(int16_t((buffer[4] << 8) | buffer[5]))};
}

Vector3f Magnetometer::getOffsets() const {
   return offsets_;
}

Vector3f Magnetometer::getGains() const {
   return gains_;
}

void Magnetometer::setOffsets(const Vector3f& offsets) {
   offsets_ = offsets;
}

void Magnetometer::setGains(const Vector3f& gains) {
   gains_ = gains;
	/*
   uint8_t regValue;
	if (gains == 0.7) {
		regValue = 0x00;
		scale = 0.62f;
	}
	else if (gains == 1) {
		regValue = 0x01;
		scale = 0.77f;
	}
	else if (gains == 1.5) {
		regValue = 0x02;
		scale = 1.03f;
	}
	else if (gains == 2) {
		regValue = 0x03;
		scale = 1.28f;
	}
	else if (gains == 3.2) {
		regValue = 0x04;
		scale = 1.89f;
	}
	else if (gains == 3.8) {
		regValue = 0x05;
		scale = 2.17f;
	}
	else if (gains == 4.5) {
		regValue = 0x06;
		scale = 2.56f;
	}
	else if (gains == 6.5) {	// not recommended
		regValue = 0x07;
		scale = 3.57f;
	}
	else
		return false;

	// Setting is in the top 3 bits of the register.
	regValue = uint8_t(regValue << 5);
	write8(ConfigurationRegisterB, regValue);

	return true;
   */
}

int Magnetometer::setMeasurementMode(uint8_t mode) {
	write8(ModeRegister, mode);
	return 0;
}

#pragma endregion functions

// ---------- Inherited functions ----------
#pragma region inherited_functions
TestResult Magnetometer::testConnection() {
	auto test = ConnectionSelfTest::create(deviceID);
	uint8_t data;
	if (!read8(IdentityRegister, data)) {
		test->errorLevel = ErrorLevel::warning;
		test->additionalInfo += " | Read error!";
		return TestResult(test);
	}

	if (data != IdentityRegisterValue)
		test->errorLevel = ErrorLevel::warning;

	return TestResult(test);
}

#pragma endregion inherited_functions

	} // end device
} // end sat
