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

#include <string>

#include "spdlog/spdlog.h"

#include "DataTypes.hpp"

namespace sat {
namespace utils {

template<LogLevel lvl>
class Logger {};

template<>
class Logger<LogLevel::info> {

   std::shared_ptr<spdlog::logger> logger;
   bool available;

public:
   explicit Logger(std::shared_ptr<spdlog::logger> logger, bool available) : logger(std::move(logger)), available(available) { }
   
   const Logger<LogLevel::info>& operator<<(const char* message) const noexcept {
      if (available) logger->info(message);
      return *this;
   }
};


template<>
class Logger<LogLevel::critical> {

   std::shared_ptr<spdlog::logger> logger;
   bool available;

public:
   explicit Logger(std::shared_ptr<spdlog::logger> logger, bool available) : logger(std::move(logger)), available(available) { }

   const Logger<LogLevel::critical>& operator<<(const char* message) const noexcept {
      if (available) logger->critical(message);
      return *this;
   }
};


template<>
class Logger<LogLevel::trace> {

   std::shared_ptr<spdlog::logger> logger;
   bool available;

public:
   explicit Logger(std::shared_ptr<spdlog::logger> logger, bool available) : logger(std::move(logger)), available(available) { }

   const Logger<LogLevel::trace>& operator<<(const char* message) const noexcept {
      if (available) logger->trace(message);
      return *this;
   }
};



template<>
class Logger<LogLevel::warn> {

   std::shared_ptr<spdlog::logger> logger;
   bool available;

public:
   explicit Logger(std::shared_ptr<spdlog::logger> logger, bool available) : logger(std::move(logger)), available(available) { }

   const Logger<LogLevel::warn>& operator<<(const char* message) const noexcept {
      if (available) logger->warn(message);
      return *this;
   }
};


template<>
class Logger<LogLevel::debug> {

   std::shared_ptr<spdlog::logger> logger;
   bool available;

public:
   explicit Logger(std::shared_ptr<spdlog::logger> logger, bool available) : logger(std::move(logger)), available(available) { }

   const Logger<LogLevel::debug>& operator<<(const char* message) const noexcept {
      if (available) logger->debug(message);
      return *this;
   }
};


template<>
class Logger<LogLevel::err> {

   std::shared_ptr<spdlog::logger> logger;
   bool available;

public:
   explicit Logger(std::shared_ptr<spdlog::logger> logger = nullptr, bool available = false) 
      : logger(std::move(logger)), available(available) { }

   const Logger<LogLevel::err>& operator<<(const char* message) const noexcept {
      if (available) logger->error(message);
      return *this;
   }
};


template<LogLevel level>
const Logger<level>& operator<<(const Logger<level>& logger, const std::string& message) noexcept {
   return logger << message.c_str();
}


/**
 *	@brief This class is a wrapper for (some functionalities of) spdlog.
 *	It has the same functionalities of an async spdlogger avoiding errors and exceptions.
 *	If the spdlogger can't be initialized it fails silently and all
 *	logging functions can be called with no harm (but also with no result).
 */
class Log {

public:

   static Logger<LogLevel::info>     info;
   static Logger<LogLevel::critical> critical;
   static Logger<LogLevel::debug>    debug;
   static Logger<LogLevel::err>      err;
   static Logger<LogLevel::trace>    trace;
   static Logger<LogLevel::warn>     warn;

   static const std::string defaultLogFilePath;

	static void setLogFilePath(std::string fileName);

	/**
	 *	@brief Sets all loggers' level
	 */
	static void setLevel(LogLevel level);

   static void init(const std::string& filePath = defaultLogFilePath);

   static bool isOpen();

   Log() = delete;

protected:
	static bool logAvailable;
};

}
}

using sat::utils::Log;