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

#pragma once

#include "devices/Interfaces.hpp"
#include "devices/DeviceI2C.hpp"

#include "utils/DataTypes.hpp"

namespace sat {

namespace utils { struct MagnetometerSettings; }

namespace device {

/**
 *	@brief Class for the Sparkfun HMC5843 magnetometer (https://www.sparkfun.com/products/retired/9371)
 */
class Magnetometer : public DeviceI2C, public ISensor {

public:

	static const std::string DEFAULT_DEV_ID;
	static const uint8_t     DEFAULT_I2C_ADDR = 0x1E;

   /**
    * @brief best way to instantiate a Magnetometer object
    * 
    * Static function that returns a managed pointer to a Magnetometer object.
    * See this class' constructors for futher details.
    */
   static std::unique_ptr<Magnetometer> create(const utils::MagnetometerSettings& settings);

   /**
    * @brief default constructor
    */
	Magnetometer();

   /**
    * @brief class constructor
    */
	Magnetometer(const utils::MagnetometerSettings& settings);

	/**
	 *	@brief Gives the magnetometer reading in Gauss along the three axis
	 */
	std::vector<double> read() const override;

	/**
	 *	@brief Gives the magnetometer reading in Gauss along the three axis
	 */
	utils::Vector3f readMagnetometer() const;

   /**
	 *	@brief Reading function, does not apply gains and offsets
	 *	@return Raw readings for X, Y, Z axis in this order
	 */
	utils::Vector3f readMagnetometerRaw() const;

	int setMeasurementMode(uint8_t mode);

   /**
    * @retrun offsets by which each measure is translated 
    *         before being multiplyied by the gains
    */
   utils::Vector3f getOffsets() const;

   /**
    * @return gains applyied to every measurement
    */
   utils::Vector3f getGains() const;

   /**
    * @param offsets 
    */
   void setOffsets(const utils::Vector3f& offsets);

   /**
    * @param gains
    */
	void setGains(const utils::Vector3f& gains);

   /**
	 *	@brief Tests if the device is connected
	 */
	TestResult testConnection() override;

private:
	//float scale;
	utils::Vector3f offsets_;
   utils::Vector3f gains_;
};


using MagnetometerSensorPtr = std::unique_ptr<Magnetometer>;

} // namespace device
} // namespace sat