/*====================================================================
Nano satellite controller

// Copyright: Copyright (c) 2017, Alberto Cenzato
			  All rights reserved.

// Licence: BSD

// Based on: Adafruit_MCP9808.cpp by K.Townsend (Adafruit Industries)
			 http://www.adafruit.com/products/1782

Redistribution and use in source and binary forms are permitted provided that
the above copyright notice and this paragraph are duplicated in all such
forms and that any documentation, advertising materials, and other materials
related to such distribution and use acknowledge that the software was developed
by the <organization>. The name of the <organization> may not be used to endorse
or promote products derived from this software without specific prior written
permission.
THIS SOFTWARE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTIES OF MERCHANTABILITY
AND FITNESS FOR A PARTICULAR PURPOSE.

//==================================================================== */

#include "devices/TemperatureSensor.hpp"

#include <limits>

#include "utils/Settings.hpp"
#include "self_test/SelfTest.hpp"


namespace sat {
namespace device {

const string TemperatureSensor::DEFAULT_DEV_NAME = "Adafruit_MCP9808";

//	---------- constructors ------------------------
#pragma region constructors

TemperatureSensor::TemperatureSensor() : DeviceI2C() { }

TemperatureSensor::TemperatureSensor(const utils::TemperatureSensorSettings& settings)
	: DeviceI2C(settings.deviceID, settings.address) {
	
	testsAndActions.emplace_back(std::bind(&TemperatureSensor::testValues, this),
										  std::bind(&TemperatureSensor::noAction, this));
}

#pragma endregion constructors


// ---------- public member functions ---------------
#pragma region functions

vector<double> TemperatureSensor::read() const {
	return { readTemperature() };
}


float TemperatureSensor::readTemperature() const {

	if (!available)
		return numeric_limits<float>::quiet_NaN();

	uint16_t t;
	read16(MCP9808_REG_AMBIENT_TEMP, t);

	auto temp = float(t & 0x0FFF);
	temp /= 16.0f;
	if (t & 0x1000) temp -= 256;

	return temp;
}


int TemperatureSensor::shutdown_wake(uint8_t sw_ID) {
	uint16_t conf_register, conf_shutdown;
	read16(MCP9808_REG_CONFIG, conf_register);
	if (sw_ID == 1) {
		conf_shutdown = conf_register | MCP9808_REG_CONFIG_SHUTDOWN;
		write16(MCP9808_REG_CONFIG, conf_shutdown);
	}
	if (sw_ID == 0) {
		conf_shutdown = conf_register ^ MCP9808_REG_CONFIG_SHUTDOWN;
		write16(MCP9808_REG_CONFIG, conf_shutdown);
	}

	return 0;
}

std::unique_ptr<TemperatureSensor> TemperatureSensor::create(const utils::TemperatureSensorSettings & settings) {
	return std::make_unique<TemperatureSensor>(settings);
}
#pragma endregion functions

// -------------- Inherited functions ----------------------
#pragma region inherited_functions

TestResult TemperatureSensor::testConnection() {

	auto test = ConnectionSelfTest::create(deviceID);
	uint16_t value;
	if (!read16(MCP9808_REG_MANUF_ID, value)) {
		test->errorLevel = ErrorLevel::warning;
		test->additionalInfo += " | Read error!";
		return TestResult(test);
	}
	if (value != 0x0054) {
		test->errorLevel = ErrorLevel::warning;
		test->additionalInfo += " | Wrong manufacturer id";
	}
	if (!read16(MCP9808_REG_DEVICE_ID, value)) {
		test->errorLevel = ErrorLevel::warning;
		test->additionalInfo += " | Read error!";
		return TestResult(test);
	}
	if (value != 0x0400) {
		test->errorLevel = ErrorLevel::warning;
		test->additionalInfo += " | Wrong device id";
	}

	return TestResult(test);
}

TestResult TemperatureSensor::testValues() const {

	auto test = OutOfRangeSelfTest::create(deviceID);
	auto temp = readTemperature();
	if (temp < 0) {
		test->errorLevel = ErrorLevel::warning;
		test->additionalInfo += " | Temperature too low";
	}
	else if (temp > 80) {
		test->errorLevel = ErrorLevel::warning;
		test->additionalInfo += " | Temperature too high";
	}
	test->value = temp;

	return TestResult(test);
}
#pragma region inherited_functions

	} // end device
}	// end sat
