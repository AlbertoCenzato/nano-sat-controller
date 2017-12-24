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

#include "utils/Settings.hpp"

#include <fstream>
#include <algorithm>
#include <iostream>

#include "jsoncpp/json.h"
#include "config.h"

using std::string;
using std::vector;

namespace sat {
namespace utils {

// ----------------------------------------------------------
// ---------- base settings ---------------------------------
// ----------------------------------------------------------
#pragma region base_settings

void DeviceSettings::load(const Json::Value& value) {

	// initialize device ID
	auto& devID = value["deviceID"];
	if (devID.isNull()) {
		deviceID.clear();
	}
	else {
		deviceID = devID.asString();
	}

	// initlialize axis
	auto& ax = value["axis"];
	if (ax.isNull()) {
		axis = Axis::NONE;
	}
	else {
		auto str = ax.asString();
		std::transform(str.begin(), str.end(), str.begin(), tolower);

		if (str.compare("x") == 0)
			axis = Axis::X;
		else if (str.compare("y") == 0)
			axis = Axis::Y;
		else if (str.compare("z") == 0)
			axis = Axis::Z;
		else if (str.compare("all") == 0)
			axis = Axis::ALL;
		else
			axis = Axis::NONE;
	}
}


void DeviceI2CSettings::load(const Json::Value& value) {
	DeviceSettings::load(value);

	auto& addr = value["address"];
	if (addr.isNull()) {
		address = 0x00;
	}
	else {
		std::size_t uselessParam;	// needed to call stoi() but otherwise useless
		address = static_cast<uint8_t>(stoi(addr.asString(), &uselessParam, 16));
	}
}


void IMUSensorsSettings::load(const Json::Value& value) {

	DeviceI2CSettings::load(value);

	// initialize rotation matrix
	auto& rot = value["rotationMat"];
	if (!rot.isNull()) {					// otherwise it has already been initialized to identity
		for (auto i = 0; i < 3; ++i) {
			for (auto j = 0; j < 3; ++j) {
				rotationMat[i][j] = rot[3 * i + j].asFloat();
			}
		}
	}

	// initalize gains
	auto& g = value["gains"];
	if (g.isNull()) {
		gains.fill(1.f);
	}
	else {
      for (auto i = 0; i < 3; ++i) {
         auto gain = g[i].asFloat();
         if (gain < 0)
            std::cout << "WARNING! Negative gain loaded, the negative sign will be removed." << std::endl;
         gains[i] = std::abs(gain);
      }
		if (!rot.isNull())
			gains = abs(rotationMat.t() * gains);
	}

	//initiliaze offsets
	auto& of = value["offsets"];
	if (of.isNull()) {
		offsets.fill(0.f);
	}
	else {
		for (auto i = 0; i < 3; ++i)
			offsets[i] = of[i].asFloat();
		if (!rot.isNull())
			offsets = rotationMat.t() * offsets;
	}
}


void MotorSettings::load(const Json::Value& value) {

	DeviceI2CSettings::load(value);

	auto& n = value["num"];
	if (!n.isNull())
	   num = n.asInt();
}

#pragma endregion base_settings


// ----------------------------------------------------------
// ---------- imu settings ----------------------------------
// ----------------------------------------------------------
#pragma region imu_settings


void AccelerometerSettigs::load(const Json::Value& value) {
	return IMUSensorsSettings::load(value);
}


void GyroscopeSettings::load(const Json::Value& value) {
	return IMUSensorsSettings::load(value);
}


void MagnetometerSettings::load(const Json::Value& value) {
	IMUSensorsSettings::load(value);

	auto& s = value["scale"];
	if (!s.isNull())
	   scale = s.asFloat();
}

void PressureSensorSettings::load(const Json::Value& value) {
	return DeviceI2CSettings::load(value);
}

void SensorFusionSettings::load(const Json::Value& value) {
   auto& Kp = value["kp"];
   if (!Kp.isNull())
      kp = Kp.asFloat();

   auto& Ki = value["ki"];
   if (!Ki.isNull())
      ki = Ki.asFloat();
}


void IMUSettings::load(const Json::Value& value) {
   DeviceI2CSettings::load(value);

   accelerometer.load(value["accelerometer"]);
   gyroscope.load(value["gyroscope"]);
   magnetometer.load(value["magnetometer"]);
   pressureSensor.load(value["pressureSensor"]);
   sensorFusion.load(value["sensorFusion"]);
}


#pragma endregion imu_settings


// ----------------------------------------------------------
// ---------- derived settings ------------------------------
// ----------------------------------------------------------
#pragma region derived_settings

void CurrentSensorSettings::load(const Json::Value& value) {
	return DeviceI2CSettings::load(value);
}


void TemperatureSensorSettings::load(const Json::Value& value) {
	return DeviceI2CSettings::load(value);
}

void PowerBoardSettings::load(const Json::Value& value) {
   DeviceI2CSettings::load(value);
	if (address == 0x00)
		address = 0x0B;
}


void AlignmentSensorSettings::load(const Json::Value& value) {
	return DeviceSettings::load(value);
}

void ControllerSettings::load(const Json::Value& value) {
	auto& t = value["tolerance"];
	if (!t.isNull())
		tolerance = t.asFloat();

	auto& Kp = value["kp"];
   auto& Ki = value["ki"];
   auto& Kd = value["kd"];
	if (!Kp.isNull())
		kp = Kp.asFloat();
   if (!Ki.isNull())
      ki = Ki.asFloat();
   if (!Kd.isNull())
      kd = Kd.asFloat();
}

#pragma endregion derived_settings


// ----------------------------------------------------------
// ---------- nanosat settings ------------------------------
// ----------------------------------------------------------
#pragma region nanosat_settings

void NanoSatSettings::load(const Json::Value& value) {
	camera.load(value["camera"]);
	imu.load(value["imu"]);

	auto currSensors = value["currentSensors"];
	for (auto& currSensor : currSensors) {
		currentSensors.emplace_back(currSensor);
	}

	temperatureSensor.load(value["temperatureSensor"]);

   pwBoard.load(value["powerBoard"]);

	auto motors = value["dcMotors"];
	if (motors.isNull())
		std::cout << "Null motors" << std::endl;
	for (auto& motor : motors) {
		dcMotors.emplace_back(motor);
	}

	auto coils = value["coils"];
	for (auto& coil : coils) {
		this->coils.emplace_back(coil);
	}

	controller.load(value["controller"]);
}


#pragma endregion nanosat_settings



//-----------------------------------------------------------
//---------- global settings --------------------------------
//-----------------------------------------------------------
#pragma region global_settings

const string GlobalSettings::DEFAULT_LOG_FILE_PATH = "/home/pi/Desktop/nanosat.log";
const string GlobalSettings::DEFAULT_LOG_LEVEL = "warn";

void GlobalSettings::load(const Json::Value& value) {
	logFilePath = value.get("logFilePath", DEFAULT_LOG_FILE_PATH).asString();

   auto level = value.get("logLevel", DEFAULT_LOG_LEVEL).asString();
   std::transform(level.begin(), level.end(), level.begin(), tolower);
   
   logLevel = LogLevel::warn;
   for (auto i = 0; i < 7; ++i) {
      if (spdlog::level::level_names[i] == level) {
         logLevel = static_cast<LogLevel>(i);
         break;
      }
   }
}


#pragma endregion global_settings


// ----------------------------------------------------------
// ------------------------ settings ------------------------
// ----------------------------------------------------------
#pragma region settings

const string Settings::DEFAULT_CONFIG_FILE = "resources/nanosat.json";

Settings::Settings(string confFile) {
   if (confFile.empty()) {
      confFile = config::getInstallDirectory() + "/" + DEFAULT_CONFIG_FILE;
   }

   try {
      std::cout << "Trying to load " << confFile << std::endl;
      load(confFile);
   } catch (const sat::utils::FileIOException& fioex) {
      std::cerr << "Unable to read file " << confFile << std::endl;
      std::cerr << fioex.what() << std::endl;
   }
}

Settings::~Settings() { }

void Settings::load(string confFile) {

	Json::CharReaderBuilder reader;
	Json::Value root;
	string errors;

	try {
		std::ifstream istream(confFile);
		if (!Json::parseFromStream(reader, istream, &root, &errors)) {
         std::cerr << errors << std::endl;
		}
		load(root);
	}
	catch (const std::exception& ex) {
      throw sat::utils::FileIOException();
	}
}


void Settings::load(const Json::Value& value) {
	auto& glob = value["global"];
	auto& sat = value["nanoSat"];

	global.load(glob);
	nanoSat.load(sat);
}


#pragma endregion settings

}	// namespace utils
}	// namespace sat