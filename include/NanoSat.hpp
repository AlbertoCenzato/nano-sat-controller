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

#pragma once

#include <vector>
#include <memory>

#include "gnublin-api/gnublin.h"

#include "devices/MotorActuator.hpp"
#include "devices/FeedbackCoil.hpp"
#include "devices/CurrentSensor.hpp"
#include "devices/TemperatureSensor.hpp"
#include "devices/IMU10DOF.hpp"
#include "devices/AlignmentSensor.hpp"
#include "devices/PowerBoard.hpp"

#include "self_test/TestResult.hpp"

#include "devices/Interfaces.hpp"
#include "ctrl/Controller.hpp"

#include "utils/Logger.hpp"

namespace sat
{


	// ----- forward declarations -----
#pragma region forward_declarations

namespace device {
	class IActuator;
	class ISensor;
	using ActuatorPtr = unique_ptr<IActuator>;
	using SensorPtr   = unique_ptr<ISensor>;
}

namespace utils {
	struct NanoSatSettings;
}

template<class In, class Out>
using FilterAlgorithmPtr = std::unique_ptr<sat::ctrl::FilterAlgorithm<In, Out>>;

template<typename T>
using ControlAlgorithmPtr = std::unique_ptr<ctrl::ControlAlgorithm<T>>;

#pragma endregion forward_declarations


/**
 * @brief This class represents the Nano Satellite itself, with all its attached devices,
 *		  divided in sensors and actuators, a controller and an IMU.
 *		  Every actuator is associated to an axis (X, Y or Z); sensors can be associated
 *		  with either one or zero or all axis.
 */
class NanoSat {

public:
	
	explicit NanoSat(const utils::NanoSatSettings& settings);
	~NanoSat();

	/**
	 * @brief Calls selfTest() on every device attached to this NanoSat
	 * @return collection of SelfTests from the devices
	 */
	TestResult selfTest();

	device::MotorActuator* getDCMotor(utils::Axis axis) const;
	device::FeedbackCoil*  getCoil(utils::Axis axis) const;

	device::CurrentSensor*	   getCurrentSensor(utils::Axis number) const;
	device::TemperatureSensor* getTemperatureSensor() const;

	device::IMU10DOF* getIMU() const;

	device::AlignmentSensor* getAlignmentSensor() const;

   device::PowerBoard* getPowerBoard() const;


	/**
	 * @brief Gets the list of actuators attached to the nano sat.
	 *			 The list can be filtered by axis.
	 * @param axis: optional filter on axis.
	 */
	vector<device::IActuator*> getActuatorsList(utils::Axis axis = utils::Axis::ALL) const;

	/**
	 * @brief Gets the list of sensors attached to the nano sat.
	 *			 The list can be filtered by axis.
	 * @param axis: optional filter on axis.
	 */
	vector<device::ISensor*> getSensorsList(utils::Axis axis = utils::Axis::ALL) const;

	/**
	 *	@brief Moves the nano sat in position (0,0,0) 
	 *			 of the absolute reference frame.
	 *	@return true if homing has been completed successfully.
	 */
	bool performHoming();

	/**
	 * @brief Moves the nano sat as specified by the operation.
	 * @param op: operation to do.
	 * @return true if op has been completed successfully.
	 */
	bool move(ctrl::Operation op);

	void setControlFunction(ControlAlgorithmPtr<utils::Vector3f>&& controlFunction);

   void setSensorFusionAlgorithm(FilterAlgorithmPtr<utils::Matrix3f3, utils::Vector4f>&& sensorFusionAlg);
	
private:

	template<typename Dev>
	class DeviceDescriptor {
	public:
		std::unique_ptr<Dev> device;
		utils::Axis axis;

		DeviceDescriptor() : DeviceDescriptor(nullptr) { }
		explicit DeviceDescriptor(std::unique_ptr<Dev>&& device, utils::Axis axis = utils::Axis::NONE) 
			: device(std::move(device)), axis(axis) { }
	};


	gnublin_i2c bus;		// i2c communication bus
	ctrl::Controller controller;

	vector<DeviceDescriptor<device::MotorActuator>> motorDesc;
	vector<DeviceDescriptor<device::FeedbackCoil>> coilDesc;
	vector<DeviceDescriptor<device::CurrentSensor>> currSensorDesc;

	DeviceDescriptor<device::TemperatureSensor> tempSensorDesc;
	DeviceDescriptor<device::IMU10DOF> imuDesc;
	DeviceDescriptor<device::AlignmentSensor> alignmentSensorDesc;
   DeviceDescriptor<device::PowerBoard> pwBoardDesc;
};


void trackMotionUntilKeyPress(const NanoSat& satellite, std::string message);

}

using sat::utils::Axis;