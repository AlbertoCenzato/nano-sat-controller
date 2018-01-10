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

#pragma once

#include "Interfaces.hpp"
#include "DeviceI2C.hpp"

#pragma region registers
#define MCP9808_REG_CONFIG             0x01

#define MCP9808_REG_CONFIG_SHUTDOWN    0x0100
#define MCP9808_REG_CONFIG_CRITLOCKED  0x0080
#define MCP9808_REG_CONFIG_WINLOCKED   0x0040
#define MCP9808_REG_CONFIG_INTCLR      0x0020
#define MCP9808_REG_CONFIG_ALERTSTAT   0x0010
#define MCP9808_REG_CONFIG_ALERTCTRL   0x0008
#define MCP9808_REG_CONFIG_ALERTSEL    0x0004
#define MCP9808_REG_CONFIG_ALERTPOL    0x0002
#define MCP9808_REG_CONFIG_ALERTMODE   0x0001

#define MCP9808_REG_UPPER_TEMP         0x02
#define MCP9808_REG_LOWER_TEMP         0x03
#define MCP9808_REG_CRIT_TEMP          0x04
#define MCP9808_REG_AMBIENT_TEMP       0x05
#define MCP9808_REG_MANUF_ID           0x06
#define MCP9808_REG_DEVICE_ID          0x07

#pragma endregion registers

namespace sat {

namespace utils {	struct TemperatureSensorSettings; }

namespace device {

/**
 * @brief Class for Adafruit MCP9808 temperature sensor
 */
class TemperatureSensor : public DeviceI2C, public ISensor {
public:

	static const std::string DEFAULT_DEV_NAME;
	static const uint8_t		 DEFAULT_I2C_ADDR = 0x18;

	TemperatureSensor();
	TemperatureSensor(const utils::TemperatureSensorSettings& settings);

	std::vector<double> read() const override;

	/**
	* @brief  Reads the 16-bit temperature register and returns the Centigrade
	*			  temperature as a float.
	*/
	float readTemperature() const;

	/**
	* @brief Set Sensor to Shutdown-State or wake up (Conf_Register BIT8)
	*			 1= shutdown / 0= wake up
	*/
	int shutdown_wake(uint8_t sw_ID);

	static std::unique_ptr<TemperatureSensor> create(const utils::TemperatureSensorSettings& settings);

	TestResult testConnection() override;
	TestResult testValues() const;

};



using TemperatureSensorPtr = std::unique_ptr<TemperatureSensor>;

} // namespace device
} // namespace sat