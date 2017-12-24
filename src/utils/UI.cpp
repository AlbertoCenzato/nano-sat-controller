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

#include "utils/UI.hpp"

#include <iostream>
#include <thread>

namespace sat {
namespace utils {
namespace ui {

   int selectFromList(const std::vector<std::string>& options) {

      const auto SIZE = options.size();

      cout << "Please choose what you want to do:" << endl;
      for (size_t i = 0; i < SIZE; ++i) {
         cout << (i + 1) << ") " << options[i] << endl;
      }

      return read<int>();
   }

   void clearConsole() {
      cout << std::string(100,'\n') << endl;
   }

void waitKey(std::string message) {
   cout << message << endl;
   std::cin.sync();
   std::cin.get();
}

void asyncWaitKey(std::atomic_bool &flag, std::string message) {
   std::thread waitingThread([&flag, message] () {
      waitKey(message);
      flag.store(true);
   });
   waitingThread.detach();
}

bool yesNoAnswer(const std::string& question) {
   cout << question << " [y/n] " << std::flush;
   const auto answer = read<char>();
   return answer == 'y' || answer == 'Y';
}

} // namespace ui
} // namespace utils
} // namespace sat