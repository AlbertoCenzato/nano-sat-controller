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
GNU License V2 for more details: https://www.gnu.org/licenses/gpl-2.0.html

//==================================================================== */

#pragma once

#include <string>
#include <vector>
#include <functional>

#include "devices/Interfaces.hpp"
#include "utils/Logger.hpp"

namespace sat {

// forward declarations
class TestResult;
using utils::Logger;
using utils::LogLevel;
//--------------------


namespace device {

class Device;

using TestFunction   = std::function<TestResult()>;
using ActionFunction = std::function<bool()>;
using TestAndAction  = std::pair<TestFunction, ActionFunction>;


/**
 * @brief This abstract class represents a generic device connected to the nano satellite.
 *		    The device can perform self-tests diagnosing its status. 
 *	@note If a subclass needs to add specific tests to be executed it should 
 *	      add a TestFunction and an ActionFunction calling addTestAndAction(test, action) in its constructor.
 *		   See Device::selfTest() for more details.
 */
class Device : virtual public IDevice {
public:

	static const std::string DEFAULT_DEV_ID;

	explicit Device(std::string deviceID = DEFAULT_DEV_ID);

	virtual ~Device() = default;

	/**
	 * @brief Safely calls all test-action pairs of functions of the device.
	 *	
	 *	For each pair the test function is called and, in case it fails
	 *	(TestResult.hasErrOrWarn() == true), the recovery action function
	 *	is called. If action function returns true the remaining tests
	 *	are executed, if false no more tests are performed.
	 * 
	 * @return contains the list of results produced by the test functions
	 */
	TestResult selfTest() noexcept override;

	/**
	 * @brief This virtual function must be overridden by subclasses.
	 *			 It should test if the device is connected. If returns an error
	 *			 the device will be set unavailable.
	 * @return test result
	 */
	virtual TestResult testConnection() = 0;
	
	/**
	 * @brief Gives the availability status of the device. To have an updated result
	 *			 selfTest() must be explicitly called before isAvailable(),
	 *			 this function does not update the status of the device.
	 * @return true if the device is connected and working
	 */
	bool isAvailable() const noexcept override;

	/**
	 *	@brief Recovery action that sets available = false
	 *	@return always false (no more tests will be executed)
	 */
	bool disableDevice() noexcept;

	/**
	 *	@brief Returns the device ID
	 */
	std::string getID() const noexcept override;

	/**
	 *	@brief Returns a string describing the device
	 */
	std::string toString() const override;

protected:

	bool available;
	std::string deviceID;

	std::vector<TestAndAction> testsAndActions;

	/**
	 *	@brief default recovery action: does nothing
	 *	@return always true
	 */
	bool noAction() const noexcept;

	
	// TODO: is perfect forwarding really necessary here?
	/**
	 * @brief Utility function that adds a test-action pair of functions to
	 *		    testsAndActions list.
	 *		  
	 * It receives two Device member functions of type TestFunction and ActionFunction.
	 *	TestFunction   is a function with no input parameters and that returns a TestResult
	 *	ActionFunction is a function with no input parameters and that returns a bool.
	 *	It is templated to allow perfect forwarding.
	 */
	template<typename T, typename A>
	void addTestAndAction(T&& testFunction, A&& actionFunction) {
		testsAndActions.emplace_back(std::bind(std::forward<T>(testFunction), this), 
									 std::bind(std::forward<A>(actionFunction), this));
	}

};


std::ostream& operator<<(std::ostream& stream, const Device& dev);

template<utils::LogLevel level>
const utils::Logger<level>& operator<<(utils::Logger<level>& logger, const Device& dev) {
   return logger << dev.toString();
}


} // namespace device
} // namespace sat