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

#include "self_test/TestResult.hpp"

using std::string;

namespace sat
{
	TestResult::TestResult() : warnings(0), errors(0) {	}

	TestResult::TestResult(const SelfTestPtr& test) : TestResult() {
		testList.push_back(test);
		if (test->errorLevel == ErrorLevel::error)
			++errors;
		if (test->errorLevel == ErrorLevel::warning)
			++warnings;
	}

	TestResult::TestResult(const std::vector<SelfTestPtr>& tests) : TestResult() {
		for (auto& test : tests) {
			if (test->errorLevel == ErrorLevel::error)
				++errors;
			else if (test->errorLevel == ErrorLevel::warning)
				++warnings;

			testList.push_back(test);
		}
	}

	TestResult::~TestResult()	{	}
	

	void TestResult::add(const SelfTestPtr& test) {
		if (test == nullptr)
			return;

		if (test->errorLevel == ErrorLevel::error)		  ++errors;
		else if (test->errorLevel == ErrorLevel::warning) ++warnings;

		testList.push_back(test);
	}

	void TestResult::add(const TestResult& results) {
		for (const auto& test : results) {
			testList.push_back(test);
		}

		errors	+= results.errors;
		warnings += results.warnings;
	}
	

	bool TestResult::hasErrors() const {
		return errors > 0;
	}

	bool TestResult::hasErrOrWarn() const {
		return (warnings + errors) > 0;
	}

	std::size_t TestResult::size() const {
		return testList.size();
	}

	std::vector<SelfTestPtr>::iterator TestResult::begin() {
		return testList.begin();
	}

	std::vector<SelfTestPtr>::const_iterator TestResult::begin() const {
		return testList.begin();
	}

	std::vector<SelfTestPtr>::iterator TestResult::end() {
		return testList.end();
	}

	std::vector<SelfTestPtr>::const_iterator TestResult::end() const {
		return testList.end();
	}

	string TestResult::toString() const {
		string str = "";
		str += "\n------------------------------\n";
		for (const auto &err : testList) {
			str += err->toString();
			str += "\n------------------------------\n";
		}
		return str;
	}


	std::ostream& operator<<(std::ostream& stream, const TestResult& testRes) {
		stream << testRes.toString();
		return stream;
	}

}
