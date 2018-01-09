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

#include "devices/DeviceI2C.hpp"

#include <sstream>

using std::string;


namespace sat {
namespace device {

// static members initialization
const std::string DeviceI2C::DEFAULT_DEV_ID = "Generic_I2C_device";
std::mutex  DeviceI2C::busMutex;
gnublin_i2c DeviceI2C::bus;


// ---------- constructors ----------
#pragma region constructors

DeviceI2C::DeviceI2C() : Device(DEFAULT_DEV_ID), address(DEFAULT_I2C_ADDR) {
   available = false;
}

DeviceI2C::DeviceI2C(const string &name, uint8_t address)
	: Device(name), address(address) { }

#pragma endregion constructors



// ---------- public member functions ----------
#pragma region publ_functions

uint8_t DeviceI2C::getAddress() const noexcept {
	return address;
}

string DeviceI2C::toString() const {
	stringstream sstream;
	sstream << " | address: 0x" << std::hex << int(address) << std::dec;

	return Device::toString() + sstream.str();
}

std::ostream& operator<<(std::ostream& stream, const DeviceI2C& dev) {
	stream << dev.toString();
	return stream;
}

#pragma endregion publ_functions



// ---------- protected member functions ----------
#pragma region prot_functions

bool DeviceI2C::write8(uint8_t regAddress, uint8_t value) {
	return write8_const(regAddress, value);
}

bool DeviceI2C::write16(uint8_t regAddress, uint16_t value) {
	return write16_const(regAddress, value);
}

bool DeviceI2C::writeX(uint8_t regAddress, uint8_t* value, int length) {
	return writeXConst(regAddress, value, length);
}

bool DeviceI2C::write8_const(uint8_t regAddress, uint8_t value) const {
   std::lock_guard<std::mutex> lock(busMutex);
   bus.setAddress(address);
   if (bus.send(regAddress, &value, 1) == -1) {
      Log::err << deviceID << ": " << bus.getErrorMessage();
      return false;
   }
   return true;
}

bool DeviceI2C::write16_const(uint8_t regAddress, uint16_t value) const {
   std::lock_guard<std::mutex> lock(busMutex);
   bus.setAddress(address);
   uint8_t buff[] = { uint8_t(value >> 8), uint8_t(value & uint16_t(0xFF)) };
   if (bus.send(regAddress, buff, 2) == -1) {
      Log::err << deviceID << ": " << bus.getErrorMessage();
      return false;
   }
   return true;
}

bool DeviceI2C::writeXConst(uint8_t regAddress, uint8_t* value, int length) const {
   std::lock_guard<std::mutex> lock(busMutex);
   bus.setAddress(address);
   if (bus.send(regAddress, value, length) == -1) {
      Log::err << deviceID << ": " << bus.getErrorMessage();
      return false;
   }
   return true;
}

bool DeviceI2C::read8(uint8_t regAddress, uint8_t& value) const {
   std::lock_guard<std::mutex> lock(busMutex);
	bus.setAddress(address);
	if (bus.receive(regAddress, &value, 1) == -1) {
		Log::err << deviceID << ": " << bus.getErrorMessage();
		return false;
	}
	return true;
}

bool DeviceI2C::read16(uint8_t regAddress, uint16_t& value) const {
   std::lock_guard<std::mutex> lock(busMutex);
   bus.setAddress(address);
   uint8_t buff[2];
   if (bus.receive(regAddress, buff, 2) == -1) {
      Log::err << deviceID << ": " << bus.getErrorMessage();
      return false;
   }
	value = uint16_t(buff[0] << 8);
	value |= buff[1];
	return true;
}

bool DeviceI2C::readX(uint8_t regAddress, uint8_t* value, int length) const {
   std::lock_guard<std::mutex> lock(busMutex);
	bus.setAddress(address);
	if (bus.receive(regAddress, value, length) == -1) {
		Log::err << deviceID << ": " << bus.getErrorMessage();
		return false;
	}
	return true;
}

#pragma endregion prot_functions


} // namespace device
} // namespace sat