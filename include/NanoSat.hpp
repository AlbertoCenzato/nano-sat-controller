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
 * @brief This class represents the nano satellite itself, with all its attached devices:
 *		  sensors, actuators, a controller and an IMU.
 *		  Every actuator is associated to an axis (X, Y or Z); sensors can be associated
 *		  either with one, zero or all axis.
 */
class NanoSat {

public:

	/**
	 * @brief Initializes all the attached devices and then calls selfTest()
	 *        logging the result if an error occurs.
	 * @param settings
	 */
	explicit NanoSat(const utils::NanoSatSettings& settings);

	~NanoSat();


	/**
	 * @brief Calls selfTest() on every device attached to this NanoSat
	 * @return result of tests
	 */
	TestResult selfTest();

	/**
	 * @param axis
	 * @return non-owning pointer to the desired motor
	 */
	device::MotorActuator* getDCMotor(utils::Axis axis) const;

	/**
	 * @param axis
	 * @return non-owning pointer to the desired coil
	 */
	device::FeedbackCoil*  getCoil(utils::Axis axis) const;

	/**
	 * @param axis
	 * @return non-owning pointer to the desired current sensor
	 */
	device::CurrentSensor*	   getCurrentSensor(utils::Axis axis) const;

	/**
	 * @return non-owning pointer to the temperature sensor
	 */
	device::TemperatureSensor* getTemperatureSensor() const;

	/**
	 * @return non-owning pointer to the IMU
	 */
	device::IMU10DOF* getIMU() const;

	/**
	 * @return non-owning pointer to the alignment sensor
	 */
	device::AlignmentSensor* getAlignmentSensor() const;

	/**
	 * @return non-owning pointer to the power board
	 */
   device::PowerBoard* getPowerBoard() const;


	/**
	 * @brief Gets the list of actuators attached to the NanoSat.
	 *			 The list can be filtered by axis.
	 * @param axis: optional filter on axis.
	 */
	vector<device::IActuator*> getActuatorsList(utils::Axis axis = utils::Axis::ALL) const;

	/**
	 * @brief Gets the list of sensors attached to the NanoSat.
	 *			 The list can be filtered by axis.
	 * @param axis: optional filter on axis.
	 */
	vector<device::ISensor*> getSensorsList(utils::Axis axis = utils::Axis::ALL) const;

	/**
	 *	@brief Moves the nano sat in position (0,0,0)
	 *			 of the fixed reference frame.
	 *	@return true if homing has been completed successfully.
	 */
	bool performHoming();

	/**
	 * @brief Moves the nano sat as specified by the operation.
	 * @param op: operation to do.
	 * @return true if op has been completed successfully.
	 */
	bool move(ctrl::Operation op);

	/**
	 * @brief sets the function to be called at every iteration of the control loop
	 * @param controlFunction
	 */
	void setControlFunction(ControlAlgorithmPtr<utils::Vector3f>&& controlFunction);

	/**
	 * @brief sets the sensor fusion function to be called at every IMU reading
	 * @param sensorFusionAlg
	 */
   void setSensorFusionAlgorithm(FilterAlgorithmPtr<utils::Matrix3f3, utils::Vector4f>&& sensorFusionAlg);
	
private:

   /**
    * @brief Device-axis pairing
    * @tparam Dev: class of the device object
    */
	template<typename Dev>
	struct AttachedDevice {
		std::unique_ptr<Dev> device;
		utils::Axis axis;

      AttachedDevice() : AttachedDevice(nullptr) { }
		explicit AttachedDevice(std::unique_ptr<Dev>&& device, utils::Axis axis = utils::Axis::NONE)
			: device(std::move(device)), axis(axis) { }
	};


	gnublin_i2c bus;		// i2c communication bus
	ctrl::Controller controller;

	vector<AttachedDevice<device::MotorActuator>> motorDesc;
	vector<AttachedDevice<device::FeedbackCoil>>  coilDesc;
	vector<AttachedDevice<device::CurrentSensor>> currSensorDesc;

   AttachedDevice<device::TemperatureSensor> tempSensorDesc;
   AttachedDevice<device::IMU10DOF> imuDesc;
   AttachedDevice<device::AlignmentSensor> alignmentSensorDesc;
   AttachedDevice<device::PowerBoard> pwBoardDesc;
};


void trackMotionUntilKeyPress(const NanoSat& satellite, std::string message);

}

using sat::utils::Axis;