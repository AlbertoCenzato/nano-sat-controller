#pragma once

#include <iostream>
#include <string>
#include <atomic>

#include <cstdio>
#include <unistd.h>
#include <termios.h>

#include "utils/Logger.hpp"

namespace sat {
namespace ui {

enum class Key {
	unknown = 0,
	q = 'q',
   a = 'a',
   s = 's',
   enter = '\n',
};

void waitKey(std::string message = "Press Enter to Continue");

void asyncWaitKey(std::atomic_bool &flag, std::string message = "Press Enter to Continue");

template<typename T>
T read() {
   T ch;
   std::cin >> ch;
   std::cin.ignore();
   return ch;
}

template<>
inline Key read() {
	struct termios old_tio;
	tcgetattr(STDIN_FILENO, &old_tio); // get the terminal settings for stdin
	auto new_tio = old_tio;

	new_tio.c_lflag &= (~ICANON & ~ECHO); // disable canonical mode (buffered i/o) and local echo

	tcsetattr(STDIN_FILENO, TCSANOW, &new_tio); // set the new settings immediately

	auto c = getchar();
	const auto key = Key(c);

	tcsetattr(STDIN_FILENO, TCSANOW, &old_tio); // restore the former settings

	return key;
}


template<utils::LogLevel level>
void logAndPrint(const utils::Logger<level> &logger, std::string message) {
	logger << message;
	std::cout << message << std::endl;
}

} // namespace ui
} // namespace sat