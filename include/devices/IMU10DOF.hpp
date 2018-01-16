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

/**
 * @brief This class represents a 10 degrees of freedom Inertial
 *        Measurement Unit. It has 4 sensors:
 *        - ADXL345  accelerometer (3 DOF)
 *        - ITG-3200 gyroscope     (3 DOF)
 *        - HMC5843  magnetometer  (3 DOF)
 *        - pressure sensor
 *        It is a virtual device composed of these 4 I2C devices
 *        
 * Every device (with the exception of pressure sensor) has an associated rotation matrix
 * in order to have all of them in the same reference frame. All their measurements (pressure
 * excluded) are fused using a sensor fusion algorithm to enhance precision
 */
class IMU10DOF : public Device, public ISensor {
public:

	static const std::string DEFAULT_DEV_NAME;


   /**
    * @brief best way to instantiate an IMU10DOF object
    * 
    * Static function that returns a managed pointer to an IMU10DOF object.
    * See this class' constructors for futher details.
    */
	static std::unique_ptr<IMU10DOF> create(const utils::IMUSettings& settings);


	IMU10DOF();

   /**
    * @breif Conncets to and initializes IMU's 4 sensors
    * 
    * @note The nano satellite should be kept still while calling
    *       this constructor since auto-calibration of accelerometer
    *       and gyroscope requires stationary conditions.
    */
	explicit IMU10DOF(const utils::IMUSettings& settings);


	/**
	 *	@brief Generic reading function, overrides ISensor::read()
	 *	@return vector of size = 10 with the following values:
	 *			[0-2] : accelerometer readings (in milli-g)
	 *			[3-5] : gyroscope readings (deg/s)
	 *			[6-8] : magnetometer readings (in mG)
	 *			[9]	: pressure readings
	 */
	std::vector<double> read() const override;

	/**
	 *	@brief Generic reading function
	 *	@return vector with the following values:
	 *			  [0-2] : accelerometer readings (in milli-g)
	 *			  [3-5] : gyroscope readings (deg/s)
	 *			  [6-8] : magnetometer readings (in mG)
	 *			  [9]	  : pressure readings
	 */
	utils::Vector10f readIMU() const;

	/**
	 *	@brief As read() and readIMU(), but the values are not multiplied
	 *			 by gains and offsets
	 */
	utils::Vector10f readIMURaw() const;

   /**
    * @brief gives the angles of the IMU wrt the fixed reference frame
    * @override IIMU::getState()
    * 
    * Calls getYawPitchRoll() and swap their order to obtain an
    * (X,Y,Z) representation.
    * 
    * @return (X,Y,Z) angles in degrees
    */
	utils::Vector3f getState();

   /**
    * @brief Updates the state filter and gives the state in quaternions.
    * @note See Sebastian O.H. Madwick report "An efficient orientation filter for
	 *		   inertial and intertial/magnetic sensor arrays", Chapter 2, Quaternion representation.
    */
	utils::Vector4f getQ();

	/**
	 * @brief Returns the Euler angles in degrees defined with the Aerospace sequence.
	 * 
	 * Calls getQ() and converts the result in Euler angles
	 */
	utils::Vector3f getEuler();


	/**
	 * @brief Returns the yaw, pitch and roll angles in degrees
	 * 
	 * Calls getQ() and converts the result in YPR representation.
	 */
	utils::Vector3f getYawPitchRoll();

   /**
    * @brief gives accelerometer measurement in IMU reference frame
    *        (does not update the filter)
    */
	utils::Vector3f readAccelerometer()    const;

   /**
    * @brief gives accelerometer measurement in IMU reference frame
    *        (does not update the filter) without applying offsets and gains
    */
	utils::Vector3f readAccelerometerRaw() const;

   /**
    * @brief gives gyroscope measurement in IMU reference frame
    *        (does not update the filter)
    */
	utils::Vector3f readGyrosope()    const;

   /**
    * @brief gives gyroscope measurement in IMU reference frame
    *        (does not update the filter) without applying offsets and gains
    */
	utils::Vector3f readGyrosopeRaw() const;

   /**
    * @brief gives magnetometer measurement in IMU reference frame
    *        (does not update the filter)
    */
	utils::Vector3f readMagnetometer()    const;
   
   /**
    * @brief gives magnetometer measurement in IMU reference frame
    *        (does not update the filter) without applying offsets and gains
    */
	utils::Vector3f readMagnetometerRaw() const;

   /**
    * @brief gives pressure sensor measurement
    */
	long readPressure()    const;
	
   /**
    * @brief gives pressure sensor measurement without applying offsets and gains
    */
   long readPressureRaw() const;

   /**
	 * @return non-owning pointer to accelerometer
	 */
	Accelerometer* getAccelerometer();

   /**
	 * @return non-owning pointer to gyroscope
	 */
	Gyroscope* getGyroscope();

   /**
	 * @return non-owning pointer to magnetometer
	 */
	Magnetometer* getMagnetometer();

   /**
	 * @return non-owning pointer to pressure sensor
	 */
	PressureSensor* getPressureSensor();

   /**
    * @brief resets sensor fusion algorithm to initial conditions 
    *        and re-calibrates accelerometer and gyroscope
    *        
    * @note do not move the nano satellite while this function is executed
    */
	void reset();

   /**
    * @brief sets the desired sensor fusion algorithm. It should be a pointer to
    *        a struct derived from sat::ctrl::FilterAlgorithm<utils::Matrix<float,3,3>, utils::Vector4f>
    */
	void setSensorFusionAlgorithm(std::unique_ptr<ctrl::IMUSensorFusionAlg>&& algorithm);


   /**
    * @brief tests if all IMU's devices are connected
    * 
    * WARNING: pressure sensor connection test disabled!
    */
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