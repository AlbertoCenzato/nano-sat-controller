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

	virtual ~SettingsBase() { }

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
	utils::RotationMat rotationMat;

   void load(const Json::Value& value) override;
};


/**
 *	@brief Settings class for Accelerometer
 */
struct AccelerometerSettigs : IMUSensorsSettings {

	AccelerometerSettigs() { }
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

	GyroscopeSettings() { }
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

	MagnetometerSettings() { }
	explicit MagnetometerSettings(const Json::Value& value) {
      MagnetometerSettings::load(value);
	}

   void load(const Json::Value& value) override;
};


/**
 *	@brief Settings class for PressureSensor
 */
struct PressureSensorSettings : DeviceI2CSettings {
	PressureSensorSettings() { }
	explicit PressureSensorSettings(const Json::Value& value) {
      PressureSensorSettings::load(value);
	}

   void load(const Json::Value& value) override;
};


struct SensorFusionSettings : SettingsBase {

   float kp = 0.5f;
   float ki = 0.1f;

   SensorFusionSettings() { }
   explicit SensorFusionSettings(const Json::Value& value) {
      SensorFusionSettings::load(value);
   }

   void load(const Json::Value& value) override;
};

/**
 *	@brief Settings class for IMU10DOF
 */
struct IMUSettings : DeviceI2CSettings {
	AccelerometerSettigs   accelerometer;
	GyroscopeSettings		  gyroscope;
	MagnetometerSettings	  magnetometer;
	PressureSensorSettings pressureSensor;
   SensorFusionSettings   sensorFusion;

	IMUSettings() { }
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

	MotorSettings() { }
	explicit MotorSettings(const Json::Value& value) {
      MotorSettings::load(value);
	}

   void load(const Json::Value& value) override;
};


/**
 *	@brief Settings class for CurrentSensor
 */
struct CurrentSensorSettings : DeviceI2CSettings {

	CurrentSensorSettings() { }
	explicit CurrentSensorSettings(const Json::Value& value) {
      CurrentSensorSettings::load(value);
	}

   void load(const Json::Value& value) override;
};


/**
 *	@brief Settings class for TemperatureSensor
 */
struct TemperatureSensorSettings : DeviceI2CSettings {
	TemperatureSensorSettings() { }
	explicit TemperatureSensorSettings(const Json::Value& value) {
      TemperatureSensorSettings::load(value);
	}

   void load(const Json::Value& value) override;
};


/**
 * @brief Settings struct for PowerBoard
 */
struct PowerBoardSettings : DeviceI2CSettings {
   PowerBoardSettings() { }
   explicit PowerBoardSettings(const Json::Value& value) {
      PowerBoardSettings::load(value);
   }

   void load(const Json::Value& value) override;
};

/**
 *	@brief Settings class for AlignmentSensor
 */
struct AlignmentSensorSettings : DeviceSettings {

	AlignmentSensorSettings() { }
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
   float kp = 10.f / 255;
   float ki = 0.f;
   float kd = 0.f;

	ControllerSettings() { }
	explicit ControllerSettings(const Json::Value& value) {
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

	NanoSatSettings() { }
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

	GlobalSettings() { }
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