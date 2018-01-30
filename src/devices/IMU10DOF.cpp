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


#include "devices/IMU10DOF.hpp"

#include <iostream>
#include <cmath>
#include <thread>

#include "utils/Settings.hpp"
#include "self_test/TestResult.hpp"
#include "ctrl/ControlAndFilter.hpp"

using namespace std::chrono;
using this_thread::sleep_for;
using sat::utils::Vector3i;
using sat::utils::Vector3f;
using sat::utils::Vector4f;
using sat::utils::Vector10f;

namespace sat {
namespace device {


const string IMU10DOF::DEFAULT_DEV_NAME = "IMU_10_DOF";

// ---------- Constructors ----------
#pragma region constructors

IMU10DOF::IMU10DOF() : Device(DEFAULT_DEV_NAME), sensorFusionAlg(ctrl::MahoneyFilter::create()) { }
	
IMU10DOF::IMU10DOF(const utils::IMUSettings & s)
	: Device(s.deviceID), accel(s.accelerometer), gyros(s.gyroscope), 
     magne(s.magnetometer), press(s.pressureSensor), accelRotMat(s.accelerometer.rotationMat), 
     gyrosRotMat(s.gyroscope.rotationMat), magneRotMat(s.magnetometer.rotationMat), 
     sensorFusionAlg(ctrl::MahoneyFilter::create()) 
{
	sleep_for(seconds(1));

	// calibrates the gyroscope and accelerometer offsets assuming the device is in a stationary situation
	sensorFusionAlg->reset();
   gyros.zeroCalibrate(128, milliseconds(5));
   accel.zeroCalibrate(128, milliseconds(5), accelRotMat.t()*utils::Axis::Z);
}


#pragma endregion constructors


// ----------- Member functions ----------
#pragma region functions

vector<double> IMU10DOF::read() const {
	auto values = readIMU();
	return std::vector<double>(begin(values),end(values));
}


Vector3f IMU10DOF::readAccelerometer() const {
	return accelRotMat * accel.readAccel();	// multiply readAccel result by rotation matrix
}

Vector3f IMU10DOF::readAccelerometerRaw() const {
	return accelRotMat * accel.readAccelRaw();	// multiply readAccel result by rotation matrix
}

Vector3f IMU10DOF::readGyrosope() const {
	return gyrosRotMat * gyros.readGyro();	// multiply readGyro result by rotation matrix
}

Vector3f IMU10DOF::readGyrosopeRaw() const {
	return gyrosRotMat * gyros.readGyroRaw();
}

Vector3f IMU10DOF::readMagnetometer() const {
   return magneRotMat * magne.readMagnetometer();  // multiply readMagnetometer result by rotation matrix
}

Vector3f IMU10DOF::readMagnetometerRaw() const {
	return magneRotMat * magne.readMagnetometerRaw();  // multiply readMagnetometer result by rotation matrix
}

long IMU10DOF::readPressure() const {
	return press.readPressure();
}

long IMU10DOF::readPressureRaw() const {
	return press.readPressureRaw();
}

Accelerometer*  IMU10DOF::getAccelerometer()  { return &accel; }
Gyroscope*	    IMU10DOF::getGyroscope()		 { return &gyros; }
Magnetometer*   IMU10DOF::getMagnetometer()	 { return &magne; }
PressureSensor* IMU10DOF::getPressureSensor() { return &press; }


void IMU10DOF::reset() {
	sensorFusionAlg->reset();
   //gyros.zeroCalibrate(128, milliseconds(5));
   accel.zeroCalibrate(128, milliseconds(5), accelRotMat.t()*utils::Axis::Z);
}

void IMU10DOF::setSensorFusionAlgorithm(unique_ptr<ctrl::IMUSensorFusionAlg>&& algorithm) {
	sensorFusionAlg = std::move(algorithm);
}

std::unique_ptr<IMU10DOF> IMU10DOF::create(const utils::IMUSettings& settings) {
	return std::make_unique<IMU10DOF>(settings);
}



Vector10f IMU10DOF::readIMU() const {

	Vector10f values;

	auto accval = readAccelerometer();
	values[0] = accval[0];
	values[1] = accval[1];
	values[2] = accval[2];

	auto gyroval = readGyrosope();
	values[3] = gyroval[0];
	values[4] = gyroval[1];
	values[5] = gyroval[2];

	auto magneticField = readMagnetometer();
	values[6] = magneticField[0];
	values[7] = magneticField[1];
	values[8] = magneticField[2];

	values[9] = readPressure();

	return values;
}

Vector10f IMU10DOF::readIMURaw() const {

	Vector10f values;
	auto acc = readAccelerometerRaw();
	values[0] = acc[0];
	values[1] = acc[1];
	values[2] = acc[2];

	auto gyroRaw = readGyrosopeRaw();
	values[3] = gyroRaw[0];
	values[4] = gyroRaw[1];
	values[5] = gyroRaw[2];

	auto magneticField = readMagnetometerRaw();
	values[6] = magneticField[0];
	values[7] = magneticField[1];
	values[8] = magneticField[2];

	values[9] = readPressureRaw();

	return values;
}

Vector3f IMU10DOF::getState() {
	const auto ypr = getYawPitchRoll();
	return { ypr[2], ypr[1], ypr[0] };
}

Vector4f IMU10DOF::getQ() {
	return (*sensorFusionAlg)( {readGyrosope()*(PI/180),  // converting to radians/sec
                               readAccelerometer(),
      Vector3f{0.f,0.f,0.f}/*readMagnetometer()*/ });
}

Vector3f IMU10DOF::getEuler() {
	const auto q = getQ(); // quaternion
	Vector3f angles;
	angles[0] =  atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1) * 180 / PI; // psi
	angles[1] = -asin (2*q[1]*q[3] + 2*q[0]*q[2]) * 180 / PI; // theta
	angles[2] =  atan2(2*q[2]*q[3] - 2*q[0]*q[1], 2*q[0]*q[0] + 2*q[3]*q[3] - 1) * 180 / PI; // phi

	return angles;
}

Vector3f IMU10DOF::getYawPitchRoll() {
	const auto q = getQ();

	// estimated gravity direction
	const auto gx = 2 * (q[1]*q[3] - q[0]*q[2]);
	const auto gy = 2 * (q[0]*q[1] + q[2]*q[3]);
	const auto gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

	Vector3f ypr;
	ypr[0] = -atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1) * 180 / PI; // psi
	ypr[1] = -atan(gx / sqrt(gy*gy + gz*gz)) * 180 / PI;
	ypr[2] = atan(gy / sqrt(gx*gx + gz*gz)) * 180 / PI;

	return ypr;
}

#pragma endregion functions


// ---------- Inherited functions ----------
#pragma region inherited_functions

// WARNING: pressure sensor connection test disabled!
TestResult IMU10DOF::testConnection() {

	TestResult result;
	auto accelRes = accel.testConnection(); // test accelerometer
	if (accelRes.hasErrOrWarn())
		accel.disableDevice();
	result.add(accelRes);

	auto gyroRes = gyros.testConnection(); // test gyroscope
	if (gyroRes.hasErrOrWarn())
		gyros.disableDevice();
	result.add(gyroRes);

	auto magnRes = magne.testConnection(); // test magnetometer
	if (magnRes.hasErrOrWarn())
		magne.disableDevice();
	result.add(magnRes);

	//res = press.testConnection(); // test pressure sensor
	//if (res.hasErrOrWarn())
	//	press.disableDevice();
	//result.add(res);

	return result;
}

#pragma endregion inherited_functions


} // namespace device
} // namespace sat
