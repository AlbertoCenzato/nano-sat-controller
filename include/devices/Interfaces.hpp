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

namespace sat
{

// forward declaration
namespace utils { template<typename T, int dim> class Vector; }

namespace device {

class IDevice {

public:

	virtual ~IDevice() { }

	/**
	 * @brief Safely calls all test-action pairs of functions of the device.
	 *			 For each pair the test function is called and, in case it fails
	 *			 (TestResult.hasErrOrWarn() == true), the recovery action function
	 *			 is called. If action function returns true the remaining tests
	 *			 are executed, if false no more tests are performed.
	 * \return contains the list of results produced by the test functions
	 */
	virtual TestResult selfTest() noexcept = 0;

	/**
	 * @brief Gives the status of the device. To have an updated result
	 *			 selfTest() must be explicitly called before isAvailable(),
	 *			 this function does not update the status of the device.
	 * \return true if the device is connected and working
	 */
	virtual bool isAvailable() const = 0;

	/**
	 *	@brief Returns the device ID
	 */
	virtual std::string getID() const = 0;

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

	virtual ~ISensor() { }

	virtual std::vector<double> read() const = 0;
};



/**
 *	@brief This interface represents a device capable of
 *			 acting in some way
 */
class IActuator : virtual public IDevice {
public:

	IActuator() { }

	virtual ~IActuator() { }

	virtual void act(float action) = 0;
};


/**
 *	@brief This interface represents a device capable of
 *			 acting in some way
 */
class IFeedbackActuator : public IActuator {
public:

	virtual ~IFeedbackActuator() { }

	// TODO: give a better name
	virtual void lowLevelFeedbackLoop(float action) = 0;
};



/**
 *	@brief This class represents an abstract IMU with an undefined number
 *			 of degrees of freedom. Probabily useless.
 */
class IIMU : public ISensor {
public:

	IIMU() : ISensor() { }
	virtual ~IIMU() { }

	/**
	 * @biref Gives the rotation around X, Y and Z axis
	 */
	virtual utils::Vector<float,3> getState() = 0;
};



typedef std::unique_ptr<ISensor>   SensorPtr;
typedef std::unique_ptr<IActuator> ActuatorPtr;



}	// namespace device
}	// namespace sat