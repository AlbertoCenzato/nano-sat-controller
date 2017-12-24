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

#include "devices/Device.hpp"

#include <inttypes.h>
#include <mutex>

#ifndef BOARD
#define BOARD RASPBERRY_PI		// this define is necessary for gnublin.h
#endif

#pragma warning(push,0)		// disable warnings from gnublin.h
#include "gnublin-api/gnublin.h"
#pragma warning(pop)

#include "utils/Logger.hpp"


namespace sat {
namespace device {

/**
 *	@brief This class adds to Device the capability of doing I\O
 *			 operations on a I2C bus			 
 */
class DeviceI2C : public Device {

public:

	static const uint8_t		 DEFAULT_I2C_ADDR = 0x00;

	DeviceI2C();
	//explicit DeviceI2C(gnublin_i2c *bus);
	DeviceI2C(const std::string &name, gnublin_i2c *bus, uint8_t address);
	virtual ~DeviceI2C();

	/**
	 *	@brief Gets the 8-bit I2C address of the device
	 */
	uint8_t getAddress() const;


	/**
	 *	@brief Returns a string describing the device
	 */
	std::string toString() const override;


protected:
   static std::mutex busMutex;
	gnublin_i2c* bus;	// I2C bus used for I\O operations
	uint8_t address;

	/**
	 *	@brief Writes 8 bits to the destination register
	 *	@param regAddress: register to write to
	 *	@param value:  value to be written
	 *	
	 *	\return false if an error occurred accessing the bus, true otherwise
	 */
	virtual bool write8 (uint8_t regAddress, uint8_t  value);

	/**
	*	@brief Writes 16 bits to the destination register
	*	@param regAddress: register to write to
	*	@param value:  value to be written
	*
	*	\return false if an error occurred accessing the bus, true otherwise
	*/
	virtual bool write16(uint8_t regAddress, uint16_t value);

	/**
	 *	@brief Writes X bits to the destination register
	 *	@param regAddress: register to write to
	 *	@param value:  array containing values to be written
	 *	@param length: number of BYTES to write
	 *
	 *	\return false if an error occurred accessing the bus, true otherwise
	 */
	virtual bool writeX (uint8_t regAddress, uint8_t* value, int length);

	/**
	 *	@brief Const version of write8 function. Take care of using this
	 *			 function only for request a reading and not for any writing
	 *			 operation that could change the device state
	 */
   bool write8_const(uint8_t regAddress, uint8_t value) const;

	/**
	 *	@brief Const version of write16 function. Take care of using this
	 *			 function only for request a reading and not for any writing
	 *			 operation that could change the device state
	 */
   bool write16_const(uint8_t regAddress, uint16_t value) const;

	/**
	 *	@brief Const version of writeX function. Take care of using this
	 *			 function only for request a reading and not for any writing
	 *			 operation that could change the device state
	 */
   bool writeXConst(uint8_t regAddress, uint8_t* value, int length) const;



	/**
	 *	@brief Reads 8 bits from the specified register into the destination variable
	 *	@param regAddress: register to read from
	 *	@param value:  output value
	 *
	 *	\return false if an error occurred accessing the bus, true otherwise
	 */
	virtual bool read8(uint8_t regAddress, uint8_t& value) const;

	/**
	 *	@brief Reads 16 bits from the specified register into the destination variable
	 *	@param regAddress: register to read from
	 *	@param value:  output value
	 *
	 *	\return false if an error occurred accessing the bus, true otherwise
	 */
	virtual bool read16(uint8_t regAddress, uint16_t& value) const;

	/**
	 *	@brief Reads X bits from the specified register into the destination variable
	 *	@param regAddress: register to read from
	 *	@param value:  output value
	 *	@param length: number of BYTES to write
	 *
	 *	\return false if an error occurred accessing the bus, true otherwise
	 */
	virtual bool readX(uint8_t regAddress, uint8_t* value, int length) const;
};


/**
 *  @brief Returns a string describing the device
 */
std::ostream& operator<<(std::ostream& stream, const DeviceI2C& dev);

template<LogLevel level>
const utils::Logger<level>& operator<<(utils::Logger<level>& logger, const DeviceI2C& dev) {
   return logger << dev.toString();
}


} // namespace device
} // namespace sat