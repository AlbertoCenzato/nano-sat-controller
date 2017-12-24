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

#include "utils/OSSignalHandler.hpp"

#include <iostream>
#include <csignal>
#include "utils/Exceptions.hpp"

namespace sat {
namespace utils {
		
void OSSignalHandler::registerSignalHandlers() {
	// register signal handlers
	std::signal(SIGABRT, signalHandler);
	std::signal(SIGFPE,  signalHandler);
	std::signal(SIGILL,  signalHandler);
	std::signal(SIGSEGV, signalHandler);
	std::signal(SIGTERM, signalHandler);
	std::signal(SIGINT,  signalHandler);
}


void OSSignalHandler::signalHandler(int signum) {
	std::cout << "Interrupt signal (" << signum << ") received.\n" << std::endl;

	/*
	if (signum == SIGINT && breakInfiniteLoop == false) {
		breakInfiniteLoop = true;
		return;
	}
	*/

   if (signum == SIGFPE) {
		throw std::overflow_error("Floating point exception");
	}

	if (signum == SIGSEGV) {
		std::cout << "Segmentation fault exception thrown" << std::endl;
		throw std::runtime_error("Segmentation fault exception");
	}

	throw TerminateProgramException();
}


}
}
