/*====================================================================
					Nano satellite controller

// Copyright: Copyright (c) 2017, Alberto Cenzato
All rights reserved.

// Licence: GNU

// Based on: IMU10DOF.cpp by Fabio Varesano <fabio at varesano dot net>,
Universita' degli Studi di Torino

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU License.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU License V3 for more details: https://www.gnu.org/licenses/gpl-3.0.html

//==================================================================== */

#pragma once

#include "devices/Accelerometer.hpp"
#include "devices/Gyroscope.hpp"
#include "devices/Magnetometer.hpp"
#include "devices/PressureSensor.hpp"

#include "utils/DataTypes.hpp"
#include "ctrl/ControlAndFilter.hpp"


namespace sat  {

namespace utils { struct IMUSettings; }


namespace device {

class IMU10DOF : public DeviceI2C, public IIMU {
public:

	static const std::string DEFAULT_DEV_NAME;
	static const uint8_t	 DEFAULT_I2C_ADDR = 0xFF;	// virtual address given to IMU to figure as a sensor

	IMU10DOF();
	IMU10DOF(gnublin_i2c *bus, const utils::IMUSettings& settings);


	/**
	 *	@brief Generic reading function, overrides ISensor::read()
	 *	@return vector of size = 10 with the following values:
	 *			[0-2] : accelerometer readings (in milli-g)
	 *			[3-5] : gyroscope readings // TODO: add mesurement unit
	 *			[6-8] : magnetometer readings (in mG)
	 *			[9]	  : pressure readings  // TODO: add mesurement unit
	 */
	std::vector<double> read() const override;

	/**
	 *	@brief Generic reading function, overrides ISensor::read()
	 *	@return vector of size = 10 with the following values:
	 *			  [0-2] : accelerometer readings (in milli-g)
	 *			  [3-5] : gyroscope readings // TODO: add mesurement unit
	 *			  [6-8] : magnetometer readings (in mG)
	 *			  [9]	  : pressure readings  // TODO: add mesurement unit
	 */
	utils::Vector10f readIMU() const;

	/**
	 *	@brief As read() and readIMU(), but the values are not multiplied
	 *			 by gains and offsets
	 */
	utils::Vector10f readIMURaw() const;

	utils::Vector3f getState() override;
	utils::Vector4f getQ();

	/**
	 * @brief Returns the Euler angles in degrees defined with the Aerospace sequence.
	 *		  See Sebastian O.H. Madwick report "An efficient orientation filter for
	 *		  inertial and intertial/magnetic sensor arrays", Chapter 2, Quaternion representation.
	 */
	utils::Vector3f getEuler();


	/**
	 * @brief Returns the yaw, pitch and roll angles in degrees
	 */
	utils::Vector3f getYawPitchRoll();

	utils::Vector3f readAccelerometer()    const;
	utils::Vector3f readAccelerometerRaw() const;
	utils::Vector3f readGyrosope()    const;
	utils::Vector3f readGyrosopeRaw() const;
	utils::Vector3f readMagnetometer()    const;
	utils::Vector3f readMagnetometerRaw() const;
	long readPressure()    const;
	long readPressureRaw() const;


	Accelerometer*  getAccelerometer();
	Gyroscope*	    getGyroscope();
	Magnetometer*   getMagnetometer();
	PressureSensor* getPressureSensor();

   //void startReading() override;
	void reset();

	void setSensorFusionAlgorithm(std::unique_ptr<ctrl::IMUSensorFusionAlg>&& algorithm);

	static std::unique_ptr<IMU10DOF> create(gnublin_i2c *bus, const utils::IMUSettings& settings);

	// WARNING: pressure sensor connection test disabled!
	TestResult testConnection() override;

private:

	Accelerometer  accel;
	Gyroscope	   gyros;
	Magnetometer   magne;
	PressureSensor	press;

	utils::RotationMat accelRotMat, gyrosRotMat, magneRotMat;

	// functor implementing a sensor fusion algorithm
	std::unique_ptr<ctrl::IMUSensorFusionAlg> sensorFusionAlg;
	
};


using IMU10DOFPtr = std::unique_ptr<IMU10DOF>;


} // namespace device
} // namespace sat