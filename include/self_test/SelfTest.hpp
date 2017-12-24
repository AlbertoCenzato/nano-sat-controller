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
#include <memory>
#include "utils/Logger.hpp"

namespace sat {

enum class ErrorLevel {
	none,
	warning,
	error
};

/**
 *	@brief This struct represents the result of a generic device self-test.
 *			 
 *	It carries a test description, the id of the tested device,
 *	the error level (if any) and some (optional) additional infos.
 *	To extend this struct simply override "static const std::string description",
 *	see examples below and in SelfTest.cpp
 */
struct SelfTest {

	static const std::string description;

	std::string deviceId;
	ErrorLevel  errorLevel;
	std::string additionalInfo;
	
	SelfTest();
	explicit SelfTest(const std::string &deviceId, ErrorLevel level = ErrorLevel::none);
	virtual ~SelfTest();

	/**
	 *	@brief Returns test description and additional info
	 */
	virtual std::string getDescription() const;

	/**
	 *	@brief Returns a detailed description of the test result
	 */
	virtual std::string toString() const;

	static std::shared_ptr<SelfTest> create(const std::string &deviceId, ErrorLevel level = ErrorLevel::none);
};


std::ostream& operator<<(std::ostream& stream, const SelfTest& selfTest);

template<utils::LogLevel level>
const utils::Logger<level>& operator<<(const utils::Logger<level>& logger, const SelfTest& selfTest) {
   return logger << selfTest.toString();
}


/**
 *	@brief Represents the result of a connection test
 */
struct ConnectionSelfTest : SelfTest {

	static const std::string description;

	explicit ConnectionSelfTest(const std::string &deviceId, ErrorLevel level = ErrorLevel::none);

	std::string getDescription() const override;

	static std::shared_ptr<ConnectionSelfTest> create(const std::string &deviceId, ErrorLevel level = ErrorLevel::none);
};


/**
 *	@brief Represents the result of a range test, which checks if values
 *			 read from the device are in a valid range (eg: temperatures
 *			 above 100°C are likely to be given by a sensor malfunctioning)
 */
struct OutOfRangeSelfTest : SelfTest {

	double value;
	static const std::string description;

	explicit OutOfRangeSelfTest(const std::string &deviceId, ErrorLevel level = ErrorLevel::none);

	std::string getDescription() const override;

	static std::shared_ptr<OutOfRangeSelfTest> create(const std::string &deviceId, ErrorLevel level = ErrorLevel::none);
};


/**
 * @brief Represents the result of an actuation test, i.e. if the actuator
 *			 produces the expected result when activated.
 */
struct ActuationSelfTest : SelfTest {

	static const std::string description;

	explicit ActuationSelfTest(const std::string &deviceId, ErrorLevel level = ErrorLevel::none);

	std::string getDescription() const override;

	static std::shared_ptr<OutOfRangeSelfTest> create(const std::string &deviceId, ErrorLevel level = ErrorLevel::none);
};


using SelfTestPtr = std::shared_ptr<SelfTest>;

} // namespace sat