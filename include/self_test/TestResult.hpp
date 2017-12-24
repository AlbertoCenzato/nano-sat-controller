/*====================================================================
						Nano satellite controller

// Copyright: Copyright (c) 2017, Alberto Cenzato
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
#include <string>

#include "self_test/SelfTest.hpp"

namespace sat {


class TestResult {
public:

	int warnings;
	int errors;
	std::vector<SelfTestPtr> testList;

	TestResult();
	explicit TestResult(const SelfTestPtr& test);
	explicit TestResult(const std::vector<SelfTestPtr>& tests);
	TestResult(TestResult&& tr) noexcept;
	virtual ~TestResult();

	TestResult& operator=(TestResult tr) noexcept;
	friend void swap(TestResult& tr1, TestResult& tr2) noexcept;

	void add(const SelfTestPtr& test);
	void add(const TestResult& results);

	bool hasErrors() const;
	bool hasErrOrWarn() const;

	std::size_t size() const;
	std::vector<SelfTestPtr>::iterator begin();
	std::vector<SelfTestPtr>::iterator end();

	std::vector<SelfTestPtr>::const_iterator begin() const;
	std::vector<SelfTestPtr>::const_iterator end() const;

	std::string toString() const;

};

std::ostream& operator<<(std::ostream& stream, const TestResult& testRes);

template<utils::LogLevel level>
const utils::Logger<level>& operator<<(const utils::Logger<level>& logger, const TestResult& testRes) {
   return logger << testRes.toString();
}

} // namespace sat