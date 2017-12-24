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

/**---------------------------------------------------------------------------
 * This file contains project-specific exceptions. 
 * If you need to define others please put them here
 * ---------------------------------------------------------------------------*/

#include <stdexcept>
//#include <locale.h>

namespace sat {
namespace utils {

class NotImplementedException : public std::logic_error {
public:
	explicit NotImplementedException(const std::string& message = "Function not yet implemented") 
      : std::logic_error(message) { }
};

class TerminateProgramException : public std::runtime_error {
public:
   explicit TerminateProgramException(const std::string& message = "Program termination request")
      : std::runtime_error(message) { }
};

class scheduling_error : public std::runtime_error {  // TODO: use std::system_error and std::error_code
public:
   explicit scheduling_error(const std::string& message = "Scheduling error") : std::runtime_error(message) { }
};

class FileIOException : public std::runtime_error {
public:
   explicit FileIOException(const std::string& message = "File I/O error") : std::runtime_error(message) { }
};

} // namespace utils
} // namespace sat