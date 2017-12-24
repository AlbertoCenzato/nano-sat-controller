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

#include "utils/Logger.hpp"

#include <iostream>

using std::string;

namespace sat {
namespace utils {

// ------------- static variables initialization ------------------------
const string Log::defaultLogFilePath = "/home/pi/Desktop/nanosat_log.txt";
bool Log::logAvailable = false;
Logger<LogLevel::info>     Log::info     = Logger<LogLevel::info>    (nullptr, false);
Logger<LogLevel::critical> Log::critical = Logger<LogLevel::critical>(nullptr, false);
Logger<LogLevel::debug>    Log::debug    = Logger<LogLevel::debug>   (nullptr, false);
Logger<LogLevel::err>      Log::err      = Logger<LogLevel::err>     (nullptr, false);
Logger<LogLevel::trace>    Log::trace    = Logger<LogLevel::trace>   (nullptr, false);
Logger<LogLevel::warn>     Log::warn     = Logger<LogLevel::warn>    (nullptr, false);

// ---------- static member functions ----------
#pragma region static_functions

void Log::setLevel(LogLevel level) {
	spdlog::set_level(level);
}


void Log::init(const string& filePath) {
   const string loggerName("async_logger");
   const std::size_t queueSize = 4096;
   std::shared_ptr<spdlog::logger> logger;
   logAvailable = true;
   try {
      spdlog::set_async_mode(queueSize);
      logger = spdlog::basic_logger_mt(loggerName, filePath);
   }
   catch (const spdlog::spdlog_ex& ex) {
      std::cout << ex.what() << std::endl;
      std::cout << "WARNING: Log unavailable!" << std::endl;
      logAvailable = false;
      logger = nullptr;
   }

   info     = Logger<LogLevel::info>    (logger, logAvailable);
   critical = Logger<LogLevel::critical>(logger, logAvailable);
   debug    = Logger<LogLevel::debug>   (logger, logAvailable);
   err      = Logger<LogLevel::err>     (logger, logAvailable);
   trace    = Logger<LogLevel::trace>   (logger, logAvailable);
   warn     = Logger<LogLevel::warn>    (logger, logAvailable);
}

bool Log::isOpen() {
   return Log::logAvailable;
}

void Log::setLogFilePath(string fileName) {
   Log::init(fileName);
}

#pragma endregion static_functions

}
}