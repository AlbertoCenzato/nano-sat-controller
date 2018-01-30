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

#include <vector>
#include <iostream>
#include <iomanip>

namespace sat {
namespace ui {

	int selectFromList(const std::vector<std::string>& options);

   std::vector<bool> multipleSelectFromList(const std::vector<std::string>& options);

   template<typename T>
   void print(const std::vector<T>& vec) {
      std::cout << "(" << std::setprecision(3);
      for (auto i = 0; i < vec.size()-1; ++i) {
         std::cout << vec[i] << ",";
      }
      std::cout << vec[vec.size() - 1] << ")" << std::endl;
   }

	void clearConsole();

   bool yesNoAnswer(const std::string& question);

} // namespace ui
} // namespace sat