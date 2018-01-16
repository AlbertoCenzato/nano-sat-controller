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

#pragma once

#include <string>
#include <vector>
#include <chrono>

#include "jsoncpp/json-forwards.h"
#include "utils/DataTypes.hpp"
#include "utils/Exceptions.hpp"

namespace sat {
namespace utils {

/**
 *	@brief Abstract Base Class for all Settings.
 *			 Every setting can be loaded from a JSON object
 */
struct SettingsBase {

	virtual ~SettingsBase() = default;

	virtual void load(const Json::Value& value) = 0;
};


/**
 *	@brief Base class for every device setting
 */
struct DeviceSettings : SettingsBase {

	std::string deviceID;
	Axis axis;

   void load(const Json::Value& value) override;
};


/**
 *	@brief Base class for every i2c device setting
 */
struct DeviceI2CSettings : DeviceSettings {

	uint8_t address;

   void load(const Json::Value& value) override;
};


/**
 *	@brief Base setting class for every sensor that composes the IMU
 */
struct IMUSensorsSettings : DeviceI2CSettings {

	Vector3f offsets;
	Vector3f gains;
	RotationMat rotationMat;

   void load(const Json::Value& value) override;
};


/**
 *	@brief Settings class for Accelerometer
 */
struct AccelerometerSettigs : IMUSensorsSettings {

	AccelerometerSettigs() = default;
	explicit AccelerometerSettigs(const Json::Value& value) {
      AccelerometerSettigs::load(value);
	}

   void load(const Json::Value& value) override;
};


/**
 *	@brief Settings class for Gyroscope
 */
struct GyroscopeSettings : IMUSensorsSettings {

	std::array<bool, 3> polarity;

	GyroscopeSettings() = default;
	explicit GyroscopeSettings(const Json::Value& value) {
      GyroscopeSettings::load(value);
	}

   void load(const Json::Value& value) override;
};


/**
 *	@brief Settings class for Magnetometer
 */
struct MagnetometerSettings : IMUSensorsSettings {

	float scale = 1.f;

	MagnetometerSettings() = default;
	explicit MagnetometerSettings(const Json::Value& value) {
      MagnetometerSettings::load(value);
	}

   void load(const Json::Value& value) override;
};


/**
 *	@brief Settings class for PressureSensor
 */
struct PressureSensorSettings : DeviceI2CSettings {
	PressureSensorSettings() = default;
	explicit PressureSensorSettings(const Json::Value& value) {
      PressureSensorSettings::load(value);
	}

   void load(const Json::Value& value) override;
};


struct SensorFusionSettings : SettingsBase {

   float kp = 0.5f;
   float ki = 0.1f;

   SensorFusionSettings() = default;
   explicit SensorFusionSettings(const Json::Value& value) {
      SensorFusionSettings::load(value);
   }

   void load(const Json::Value& value) override;
};

/**
 *	@brief Settings class for IMU10DOF
 */
struct IMUSettings : DeviceSettings {
	AccelerometerSettigs   accelerometer;
	GyroscopeSettings		  gyroscope;
	MagnetometerSettings	  magnetometer;
	PressureSensorSettings pressureSensor;
   SensorFusionSettings   sensorFusion;

	IMUSettings() = default;
	explicit IMUSettings(const Json::Value& value) {
      IMUSettings::load(value);
	}

   void load(const Json::Value& value) override;
};


/**
 *	@brief Settings class for MotorActuator
 */
struct MotorSettings : DeviceI2CSettings {
	int num = 0;

	MotorSettings() = default;
	explicit MotorSettings(const Json::Value& value) {
      MotorSettings::load(value);
	}

   void load(const Json::Value& value) override;
};


/**
 *	@brief Settings class for CurrentSensor
 */
struct CurrentSensorSettings : DeviceI2CSettings {

	CurrentSensorSettings() = default;
	explicit CurrentSensorSettings(const Json::Value& value) {
      CurrentSensorSettings::load(value);
	}

   void load(const Json::Value& value) override;
};


/**
 *	@brief Settings class for TemperatureSensor
 */
struct TemperatureSensorSettings : DeviceI2CSettings {
	TemperatureSensorSettings() = default;
	explicit TemperatureSensorSettings(const Json::Value& value) {
      TemperatureSensorSettings::load(value);
	}

   void load(const Json::Value& value) override;
};


/**
 * @brief Settings struct for PowerBoard
 */
struct PowerBoardSettings : DeviceI2CSettings {
   PowerBoardSettings() = default;
   explicit PowerBoardSettings(const Json::Value& value) {
      PowerBoardSettings::load(value);
   }

   void load(const Json::Value& value) override;
};

/**
 *	@brief Settings class for AlignmentSensor
 */
struct AlignmentSensorSettings : DeviceSettings {

	AlignmentSensorSettings() = default;
	explicit AlignmentSensorSettings(const Json::Value& value) {
      AlignmentSensorSettings::load(value);
	}

   void load(const Json::Value& value) override;
};


/**
 * @brief Settings class for Controller
 */
struct ControllerSettings : SettingsBase {
	
	float tolerance = 1.f;
   float kp = 10.f / 255;  // PID proportional gain
   float ki = 0.f;         // PID integral gain
   float kd = 0.f;         // PID derivative gain

   uint32_t measurementsPerControl = 10;
   std::chrono::milliseconds ctrlLoopTimeout; 

	ControllerSettings() : ctrlLoopTimeout(20000) { }
	explicit ControllerSettings(const Json::Value& value) : ctrlLoopTimeout(20000) {
      ControllerSettings::load(value);
	}

   void load(const Json::Value& value) override;
};


/**
 *	@brief Settings class for NanoSat
 */
struct NanoSatSettings : SettingsBase {

	AlignmentSensorSettings	  camera;
	IMUSettings				  imu;
	TemperatureSensorSettings temperatureSensor;
   PowerBoardSettings   pwBoard;

	std::vector<CurrentSensorSettings> currentSensors;
	std::vector<MotorSettings>		   dcMotors;
	std::vector<MotorSettings>		   coils;

	ControllerSettings	controller;

	NanoSatSettings() = default;
	explicit NanoSatSettings(const Json::Value& value) {
      NanoSatSettings::load(value);
	}

   void load(const Json::Value& value) override;
};


/**
 *	@brief Settings class for global settings.
 *			 Contains the path to the log file.
 */
struct GlobalSettings : SettingsBase {

	static const std::string DEFAULT_LOG_FILE_PATH;
   static const std::string DEFAULT_LOG_LEVEL;

	std::string logFilePath;
   LogLevel logLevel;

	GlobalSettings() = default;
	explicit GlobalSettings(const Json::Value& value) {
      GlobalSettings::load(value);
	}

   void load(const Json::Value& value) override;
};


/**
 *	@brief This class is used to load the program configuration json file.
 *			 It is hierarchically structured: nanoSat contains NanoSat settings,
 *			 while global contains all the other settings.
 */
struct Settings : SettingsBase {

	static const std::string DEFAULT_CONFIG_FILE;

	NanoSatSettings nanoSat;
	GlobalSettings  global;

	/**
	 *	@brief The only calss constructor
	 *	@param confFile: configuration json file path
	 */
	explicit Settings(std::string confFile = "");
	~Settings();

	/**
	 *	@brief Loads nanoSat and global from a json configuration file
	 *	@return true if confFile reading succeeded
	 * @throws sat::utils::FileIOException
	 */
	void load(std::string confFile);

	/**
	*	@brief Loads nanoSat and global from a json object
	*/
   void load(const Json::Value& value) override;
};

}  // namespace utils
}	// namespace sat