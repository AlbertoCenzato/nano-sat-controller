/*====================================================================
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

//==================================================================== */

#include "NanoSat.hpp"

#include "utils/Settings.hpp"
#include "utils/UI.hpp"


using sat::device::CurrentSensor;
using sat::device::MotorActuator;
using sat::device::IMU10DOF;
using sat::device::TemperatureSensor;
using sat::device::AlignmentSensor;
using sat::device::PowerBoard;
using sat::device::FeedbackCoil;
using sat::device::IActuator;
using sat::device::ISensor;

using sat::utils::Axis;
using sat::utils::NanoSatSettings;
using sat::utils::Vector4f;
using sat::utils::Matrix3f3;
using sat::utils::Logger;

namespace sat {

// ---------- constructors ----------
#pragma region constructors

NanoSat::NanoSat(const NanoSatSettings &settings) : controller(settings.controller) {

	//--- sensors initialization ---

	// current sensors
	Log::info << "Current sensors initialization...";
	for (const auto& currSettings : settings.currentSensors)
		currSensorDesc.emplace_back(CurrentSensor::create(currSettings), currSettings.axis);

	//temperature sensors
	Log::info << "Temperature sensor initialization...";
	auto &tempSettings = settings.temperatureSensor;
	tempSensorDesc = AttachedDevice<TemperatureSensor>(TemperatureSensor::create(tempSettings), tempSettings.axis);

	// inertial measurement unit initialization
   Log::info << "IMU initialization...";
	imuDesc = AttachedDevice<IMU10DOF>(IMU10DOF::create(settings.imu), Axis::ALL);
	
	// alignment sensor
	Log::info << "Alignment sensor initialization...";
	auto &alignSettings = settings.camera;
	alignmentSensorDesc = AttachedDevice<AlignmentSensor>(AlignmentSensor::create(alignSettings), alignSettings.axis);

   // power board
	Log::info << "Power board initialization...";
   auto &pwBoardSettings = settings.pwBoard;
   pwBoardDesc = AttachedDevice<PowerBoard>(PowerBoard::create(pwBoardSettings), Axis::NONE);


	//--- actuators initialization ---

	// dc motors
	Log::info << "DC motors initialization...";
	for (const auto& motorSettings : settings.dcMotors)
		motorDesc.emplace_back(MotorActuator::create(motorSettings), motorSettings.axis);

	// coils
	Log::info << "Coils initialization...";
	for (const auto& coilSettings : settings.coils) {
		auto axis = coilSettings.axis;
		coilDesc.emplace_back(FeedbackCoil::create(coilSettings, getCurrentSensor(axis)), axis);
	}

	//--- test all the devices ---
	auto testResult = selfTest();
	if(testResult.hasErrOrWarn())
	   Log::err << testResult;
}

NanoSat::~NanoSat() { }

#pragma endregion constructors


// ---------- public member functions ----------
#pragma region pub_functions

TestResult NanoSat::selfTest() {

	TestResult result;

	Log::info << "Testing all devices...";

   Log::info << "	Testing sensors...";
	auto sensors = getSensorsList();
	for (auto device : sensors) {
      Log::info << "	Testing " + device->getID() + "...";
		result.add(device->selfTest());
	}
   Log::info << "	Done";

   Log::info << "	Testing actuators...";
	auto actuators = getActuatorsList();
	for (auto device : actuators) {
      Log::info << "	Testing " + device->getID() + "...";
		result.add(device->selfTest());
	}
	Log::info << "	Done";
	Log::info << "Done";

	return result;
}

MotorActuator* NanoSat::getDCMotor(Axis axis) const {
	for (const auto &motor : motorDesc)
		if (motor.axis == axis)
			return motor.device.get();

	return nullptr;
}

FeedbackCoil* NanoSat::getCoil(Axis axis) const {
	for (const auto &coil : coilDesc)
		if (coil.axis == axis)
			return coil.device.get();

	return nullptr;
}

CurrentSensor* NanoSat::getCurrentSensor(Axis axis) const {
	for (const auto &currentSensor : currSensorDesc)
		if (currentSensor.axis == axis)
			return currentSensor.device.get();

	return nullptr;
}

TemperatureSensor* NanoSat::getTemperatureSensor() const {
	return tempSensorDesc.device.get();
}

IMU10DOF* NanoSat::getIMU() const {
	return imuDesc.device.get();
}

AlignmentSensor* NanoSat::getAlignmentSensor() const {
	return alignmentSensorDesc.device.get();
}

device::PowerBoard* NanoSat::getPowerBoard() const {
   return pwBoardDesc.device.get();
}


vector<IActuator*> NanoSat::getActuatorsList(Axis axis) const {
	
	vector<IActuator*> list;

	for (auto &motor : motorDesc)
		if (axis == Axis::ALL || motor.axis == axis)
			list.push_back(motor.device.get());

	for (auto &coil : coilDesc)
		if (axis == Axis::ALL || coil.axis == axis)
			list.push_back(coil.device.get());

	return list;
}

vector<ISensor*> NanoSat::getSensorsList(Axis axis) const {
	
	vector<ISensor*> list;

	for (auto &currSensor : currSensorDesc)
		if (axis == Axis::ALL || currSensor.axis == axis)
			list.push_back(currSensor.device.get());

	if (axis == Axis::ALL || tempSensorDesc.axis == axis)
		list.push_back(tempSensorDesc.device.get());

	if (axis == Axis::ALL || imuDesc.axis == axis)
		list.push_back(imuDesc.device.get());
	
	if (axis == Axis::ALL || tempSensorDesc.axis == axis)
		list.push_back(tempSensorDesc.device.get());

	if (axis == Axis::ALL || alignmentSensorDesc.axis == axis)
		list.push_back(alignmentSensorDesc.device.get());

	return list;
}

bool NanoSat::performHoming() {
   ctrl::Operation op;
   op.actuatorX = getDCMotor(Axis::X);
   op.actuatorY = getDCMotor(Axis::Y);
   op.actuatorZ = getDCMotor(Axis::Z);
   op.imu = getIMU();
   op.finalState = {0.f, 0.f, 0.f};
	
	Log::info << "Going home...";
	return move(op);
}

bool NanoSat::move(ctrl::Operation op) {
	auto success = controller.addOperation(op).run();
	controller.clearOperations(); // ensure to clear the maneuver list whatever the result of run() is

	return success;
}


void NanoSat::setControlFunction(std::unique_ptr<ctrl::ControlAlgorithm<utils::Vector3f>>&& controlFunction) {
	controller.setControlFunction(std::move(controlFunction));
}

void NanoSat::setSensorFusionAlgorithm(FilterAlgorithmPtr<Matrix3f3, Vector4f>&& sensorFusionAlg) {
   imuDesc.device->setSensorFusionAlgorithm(std::move(sensorFusionAlg));
}

#pragma endregion pub_functions


void trackMotionUntilKeyPress(const NanoSat& satellite, std::string message) {
   auto imu = satellite.getIMU();
   //imu->startReading();
   std::atomic_bool stop(false);
   utils::ui::asyncWaitKey(stop, message);
   while (!stop) {
      const auto state = imu->getState();
      utils::ui::clearConsole();
      std::cout << "(X, Y, Z): " << state << std::endl;
      this_thread::sleep_for(chrono::milliseconds(100));
   }
}

}
