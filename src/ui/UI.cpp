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

#include "ui/UI.hpp"

#include <iostream>
#include <ostream>
#include <thread>

#include "ui/IO.hpp"


using std::string;
using std::vector;
using std::cout;
using std::endl;
using std::cerr;

namespace sat {
namespace ui {

   enum class ColorCode {
      RESET       = 0,
      UNDERLINE   = 4,
      REVERSE     = 7,
      FG_RED      = 31,
      FG_GREEN    = 32,
      FG_BLUE     = 34,
      FG_DEFAULT  = 39,
      BG_RED      = 41,
      BG_GREEN    = 42,
      BG_BLUE     = 44,
      BG_DEFAULT  = 49
    };

    class ColorModifier {
        ColorCode code;
    public:
        ColorModifier(ColorCode pCode) : code(pCode) {}
        friend std::ostream&
        operator<<(std::ostream& os, const ColorModifier& mod) {
            return os << "\033[" << static_cast<int>(mod.code) << "m";
        }
    };


   int selectFromList(const vector<string>& options) {

      const auto SIZE = options.size();

      cout << "Please choose what you want to do:" << endl;
      for (size_t i = 0; i < SIZE; ++i) {
         cout << (i + 1) << ") " << options[i] << endl;
      }

      return read<int>();
   }

   vector<bool> multipleSelectFromList(const vector<string>& options) {
      const ColorModifier reset(ColorCode::RESET);
      const ColorModifier selected(ColorCode::BG_GREEN);
      const ColorModifier highlight(ColorCode::REVERSE);

      const auto numOfOptions = options.size();
      vector<bool> selections(numOfOptions, false);

      int index = 0;
      bool exit = false;
      while (!exit) {
         clearConsole();

         cout << "Select the highlighted option pressing [s], move up and down with [q] and [a]." << endl;
         cout << "When done press [ENTER].\n" << endl;
         for (size_t i = 0; i < numOfOptions; ++i) {
				if (!selections[i] && i != index)
					cout << reset;
				else {
					if (selections[i]) cout << selected;
					if (i == index)    cout << highlight;
				}

            cout << options[i] << reset << endl;
         }

         const auto key = read<Key>();
         if      (key == Key::a)     index = (index + 1) % numOfOptions;
         else if (key == Key::q)     index = (index - 1) % numOfOptions;
         else if (key == Key::s)     selections[index] = !selections[index];
         else if (key == Key::enter) exit = true;
      }

      return selections;
   }

   void clearConsole() {
      cout << std::string(100,'\n') << endl;
   }

bool yesNoAnswer(const std::string& question) {
   cout << question << " [y/n] " << std::flush;
   const auto answer = read<char>();
   return answer == 'y' || answer == 'Y';
}

} // namespace ui
} // namespace sat