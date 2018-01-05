/*===========================================================================
							Nano satellite controller

// Copyright: Copyright (c) 2017, Alberto Cenzato
				  All rights reserved.

// Licence: BSD

// Based on: Adafruit INA219 breakout board library 
				 by K. Townsend (Adafruit Industries)
				 https://github.com/adafruit/Adafruit_INA219

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

//============================================================================ */

#pragma once

#include "devices/Interfaces.hpp"
#include "devices/DeviceI2C.hpp"


namespace sat {

namespace utils { struct CurrentSensorSettings; }

namespace device {


/**
 *	@brief Class for INA219 current sensor
 */
class CurrentSensor : public DeviceI2C, public ISensor {
public:

	static const std::string DEFAULT_DEV_NAME;
	static const uint8_t	 DEFAULT_I2C_ADDR = 0x40;    // 1000000 (A0+A1=GND)

	CurrentSensor();
	CurrentSensor(const utils::CurrentSensorSettings& settings);

	/**
	 *	@brief Generic reading function, overrides ISensor::read()
	 *	@return vector of size = 1 with current value in mA
	 */
	std::vector<double> read() const override;

	/**
	 * @brief Gets the current value in mA, taking into account the
	 *			 config settings and current LSB
	 */
	float readCurrent() const;

	/**
	 * @brief Configures to INA219 to be able to measure up to 32V and 2A
	 *			 of current.  Each unit of current corresponds to 100uA, and
	 *			 each unit of power corresponds to 2mW. Counter overflow
	 *			 occurs at 3.2A.
	 *
	 * @note  These calculations assume a 0.1 ohm resistor is present
	 */
	void setCalibration_32V_2A();


	/**
	 * @brief Configures to INA219 to be able to measure up to 32V and 1A
	 *			 of current.  Each unit of current corresponds to 40uA, and each
	 *			 unit of power corresponds to 800�W. Counter overflow occurs at
	 *			 1.3A.
	 *
	 * @note  These calculations assume a 0.1 ohm resistor is present
	 */
	void setCalibration_32V_1A();
	void setCalibration_16V_400mA();

	/**
	 * @brief  Gets the shunt voltage in volts
	 */
	float getBusVoltage_V() const;

	/**
	 * @brief Gets the shunt voltage in mV (so +-327mV)
	 */
	float getShuntVoltage_mV() const;

	static std::unique_ptr<CurrentSensor> create(const utils::CurrentSensorSettings& settings);

	TestResult testConnection() override;
	TestResult testValues() const;

private:

	uint16_t ina219_calValue;
	// The following multipliers are used to convert raw current and power
	// values to mA and mW, taking into account the current config settings
	uint32_t ina219_currentDivider_mA;
	uint32_t ina219_powerDivider_mW;

	static const std::chrono::seconds DEVICE_RESET_TIME;

	/**
	 * @brief Overrides default DeviceI2C::read16() to wait for
	 *			 register value loading
	 */
	bool read16(uint8_t regAddress, uint16_t& value) const override;

};


using CurrentSensorPtr = std::unique_ptr<CurrentSensor>;


} // namespace device
} // namespace sat