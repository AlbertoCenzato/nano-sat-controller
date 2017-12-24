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

namespace sat {
namespace utils {

/**
 * @brief This class handles operating system interrupt signals.
 *			 
 * It is composed only of one static public member function
 * and a private signalHandler.
 * 
 * @note remember to wrap everything inside main() in a try-catch block
 * otherwise OSSignalHandler is not guaranteed to work properly.
 */
class OSSignalHandler {
public:

	/**
	 *	@brief Call this function in one of the first lines of the program
	 *		   to handle signals raised from the operating system. 
	 *		   
	 *	After this function is called all OS signals are translated into exceptions
	 *	with the following criteria: 
	 *	SIGFPE  -> std::overflow_error
	 *	SIGSEGV -> std::runtime_error
	 *	all the others -> sat::utils::TerminateProgramException
	 */
	static void registerSignalHandlers();

private:

   /**
   * @brief Handles the signals raised from the operating system
   * 
   * @throws std::overflow_error, 
   *         std::runtime_error,
   *         sat::utils::TerminateProgramException;
   *         
   * @note Exception throwing inside a signal handler is bad practice
   *       but as far as I know it is the only way of shutting down
   *       cleanly, i.e. shutting down asynchronously running DC motors
   */
	static void signalHandler(int signum);
};


} // namespace utils
} // namespace sat