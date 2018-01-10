/*===========================================================================
                           Nano satellite controller

// Copyright: Copyright (c) 2017, Alberto Cenzato
              All rights reserved.

// Licence: GNU

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU License.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU License V3 for more details: https://www.gnu.org/licenses/gpl-3.0.html

//===========================================================================*/

#pragma once

#include "devices/Interfaces.hpp"
#include "devices/DeviceI2C.hpp"

namespace sat {

namespace utils { struct PressureSensorSettings; }

namespace device {

/**
 * @brief class for BMP085 pressure sensor
 */
class PressureSensor : public DeviceI2C, public ISensor {

public:

	static const std::string DEFAULT_DEV_ID;
	static const uint8_t		 DEFAULT_I2C_ADDR = 0x77;  // default I2C address of BMP085
	static const unsigned char OSS = 0;  // Oversampling Setting

	// Calibration values
	uint16_t ac1 = 0;	// WARNING originally int, not uint
	uint16_t ac2 = 0;	// WARNING originally int, not uint
	uint16_t ac3 = 0;	// WARNING originally int, not uint
	uint16_t ac4 = 0;
	uint16_t ac5 = 0;
	uint16_t ac6 = 0;
	uint16_t b1  = 0;  // WARNING originally int, not uint
	uint16_t b2  = 0;  // WARNING originally int, not uint
	uint16_t mb  = 0;  // WARNING originally int, not uint
	uint16_t mc  = 0;  // WARNING originally int, not uint
	uint16_t md  = 0;  // WARNING originally int, not uint


   /**
    * @brief best way to instantiate a PressureSensor object
    * 
    * Static function that returns a managed pointer to a PressureSensor object.
    * See this class' constructors for futher details.
    */
   static std::unique_ptr<PressureSensor> create(const utils::PressureSensorSettings& settings);

   /**
    * @brief default constructor
    */
	PressureSensor();

   /**
    * @brief class constructor
    */
	PressureSensor(const utils::PressureSensorSettings& settings);

	~PressureSensor() = default;

   /**
	 *	@brief Generic reading function, overrides ISensor::read()
	 *	@return vector of size = 1 with pressure value
	 */
	std::vector<double> read() const override;

   /**
	 *	@brief Reading function
	 *	@return pressure value
	 */
	long readPressure() const;

   /**
    * @brief Reading function
    * @return pressure value
    */
	long readPressureRaw() const;

   /**
	 *	@brief Tests if the device is connected
	 */
	TestResult testConnection() override;

private:

	/**
	* @brief Calculate temperature given ut.
	*			 Value returned will be in units of 0.1 deg C
	*/
	long get_temperature(unsigned int ut) const;

	/**
	* @brief Calculate pressure given up. Calibration values must be known.
	*			 b5 is also required so bmp085GetTemperature(...) must be called first.
	*			 Value returned will be pressure in units of Pa.
	*/
	long get_pressure(unsigned long up) const;
	
	/**
	 *	@brief Read the uncompensated temperature value
	 */
	unsigned int readUT() const;

	/**
	 *	@brief Read the uncompensated pressure value
	 */
	unsigned long readUP() const;

};


using PressureSensorPtr = std::unique_ptr<PressureSensor>;

} // namespace device
} // namespace sat