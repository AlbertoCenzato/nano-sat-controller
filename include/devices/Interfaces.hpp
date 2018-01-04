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

#include "self_test/TestResult.hpp"

/* ----------------------------------------------------------------------
 * - This file contains all the interfaces (ABCs) needed by the devices -
 * ----------------------------------------------------------------------*/

namespace sat {

// forward declaration
namespace utils { template<typename T, int dim> class Vector; }

namespace device {


/**
 * @brief A generic device with a device ID and the possibility to 
 *        perform self tests
 */
class IDevice {

public:

	virtual ~IDevice() = default;

	/**
	 * @brief safely executes all tests associated to the device 
	 *        and returns their results
	 */
	virtual TestResult selfTest() noexcept = 0;

	/**
	 * @brief Gives the availability status of the device.
	 * @return true if the device is connected and working.
	 */
	virtual bool isAvailable() const noexcept = 0;

	/**
	 *	@brief Returns the device ID
	 */
	virtual std::string getID() const noexcept = 0;

	/**
	 *	@brief Returns a string describing the device
	 */
	virtual std::string toString() const = 0;
};



/**
 *	@brief This interface represents a device capable of 
 *			 reading some value 
 */
class ISensor : virtual public IDevice {
public:

	virtual ~ISensor() = default;

	virtual std::vector<double> read() const = 0;
};



/**
 *	@brief This interface represents a device capable of
 *			 acting in some way
 */
class IActuator : virtual public IDevice {
public:

	IActuator() = default;  // TODO: remove
	virtual ~IActuator() = default;

	virtual void act(float action) = 0;
};


/**
 *	@brief This interface represents a device capable of
 *			 running a low level feedback loop while acting
 */
class IFeedbackActuator : public IActuator {
public:

	virtual ~IFeedbackActuator() = default;

	// TODO: give a better name
	virtual void lowLevelFeedbackLoop(float action) = 0;
};



/**
 *	@brief This class represents an abstract IMU with an undefined number
 *			 of degrees of freedom. Probabily useless.
 */
class IIMU : public ISensor {
public:

	IIMU() = default;
	virtual ~IIMU() = default;

	/**
	 * @biref Gives the rotation around X, Y and Z axis
	 */
	virtual utils::Vector<float,3> getState() = 0;
};



typedef std::unique_ptr<ISensor>   SensorPtr;
typedef std::unique_ptr<IActuator> ActuatorPtr;

}	// namespace device
}	// namespace sat