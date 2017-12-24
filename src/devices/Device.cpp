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

#include "devices/Device.hpp"

#include "self_test/TestResult.hpp"

using std::string;

namespace sat {
namespace device {

const string Device::DEFAULT_DEV_ID = "Generic_device";

Device::Device(const string &deviceId) : available(true), deviceID(deviceId) {
	addTestAndAction(&Device::testConnection, &Device::disableDevice);
}

Device::~Device() { }

TestResult Device::selfTest() noexcept {
	TestResult result;
	try {
		for (auto& testAndAction : testsAndActions) {
			auto res = testAndAction.first();
			result.add(res);
			if (res.hasErrOrWarn()) {
				bool continueTesting = testAndAction.second();
				if (!continueTesting) {
					break;
				}
			}
		}
			
	}
	catch (const std::bad_function_call& ex) {
		Log::err << "ERROR! Bad test function call for " + deviceID + "!";
      Log::err << ex.what();
      Log::err << "\nDevice disabled.";

		available = false;
	}
	catch (const std::exception& ex) {
      Log::err << "ERROR! Something went wrong testing the device!";
		Log::err << "Exception thrown: ";
		Log::err << ex.what();
		Log::err << "\nDevice disabled.";

		available = false;
	}
	return result;
}

bool Device::noAction() const {
	return true;
}


bool Device::isAvailable() const {
	return available;
}

bool Device::disableDevice() {
	available = false;
	return false;
}

std::string Device::getID() const {
	return deviceID;
}

string Device::toString() const {
	return deviceID + " | available: " + (available ? "Y" : "N");
}

std::ostream& operator<<(std::ostream& stream, const Device& dev) {
	stream << dev.toString();
	return stream;
}

} // namespace device
} // namespace sat