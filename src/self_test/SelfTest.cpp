/*====================================================================
Nano satellite controller

// Copyright   : Copyright (c) 2017, Alberto Cenzato
All rights reserved.

// Licence: GNU

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU License.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU License V3 for more details: https://www.gnu.org/licenses/gpl-3.0.html

//==================================================================== */

#include "self_test/SelfTest.hpp"
#include <iomanip>

using std::string;

namespace sat {


// -------------- SelfTest ----------------------------------

const string SelfTest::description = "Generic test";

SelfTest::SelfTest() {
	errorLevel = ErrorLevel::none;
}

SelfTest::SelfTest(const string &deviceId, ErrorLevel level) {
	this->errorLevel = level;
	this->deviceId = deviceId;
}

SelfTest::~SelfTest() {}


string SelfTest::getDescription() const {
	return description;
}

string SelfTest::toString() const {
	string str;
	str += string("Device: ") + deviceId + "\n";
	str += string("Description: ") + getDescription() + string("\nAdditional info: ") + additionalInfo + "\n";
	str += string("Test: ") + (errorLevel == ErrorLevel::none ? "Passed" : "Failed") + "\n";
	str += "Error level: ";
	switch (errorLevel) {
	case ErrorLevel::warning:
		str += "warning\n";
		break;
	case ErrorLevel::error:
		str += "error\n";
		break;
	default:
		str += "zero\n";
		break;
	}
	return str;
}

std::ostream& operator<<(std::ostream& stream, const SelfTest& selfTest) {
	stream << selfTest.toString();
	return stream;
}

std::shared_ptr<SelfTest> SelfTest::create(const std::string& deviceID, ErrorLevel level) {
	return std::make_shared<SelfTest>(deviceID, level);
}


	// -------------- ConnectionError ---------------------------

const string ConnectionSelfTest::description = "Connection test. Can reach the device?";

ConnectionSelfTest::ConnectionSelfTest(const string & deviceId, ErrorLevel level) 
	: SelfTest(deviceId, level) { }

string ConnectionSelfTest::getDescription() const {
	return description;
}

std::shared_ptr<ConnectionSelfTest> ConnectionSelfTest::create(const string & deviceId, ErrorLevel level) {
	return std::make_shared<ConnectionSelfTest>(deviceId, level);
}

// -------------- OutOfRangeError ---------------------------

const string OutOfRangeSelfTest::description = "Reading test. Is read value in the expected range?";

OutOfRangeSelfTest::OutOfRangeSelfTest(const string & deviceId, ErrorLevel level) 
	: SelfTest(deviceId, level), value(0) { }

string OutOfRangeSelfTest::getDescription() const {
	return description;
}

std::shared_ptr<OutOfRangeSelfTest> OutOfRangeSelfTest::create(const std::string & deviceId, ErrorLevel level) {
	return std::make_shared<OutOfRangeSelfTest>(deviceId, level);
}

// -------------- ActuationError ---------------------------

const string ActuationSelfTest::description = "Actuation test. Actuator behaves as expected?";

ActuationSelfTest::ActuationSelfTest(const string & deviceId, ErrorLevel level) 
	: SelfTest(deviceId, level) { }

string ActuationSelfTest::getDescription() const {
	return description;
}

std::shared_ptr<OutOfRangeSelfTest> ActuationSelfTest::create(const std::string & deviceId, ErrorLevel level) {
	return std::make_shared<OutOfRangeSelfTest>(deviceId, level);
}


}	// namespace sat