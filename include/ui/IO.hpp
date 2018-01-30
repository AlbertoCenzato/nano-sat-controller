#pragma once

#include <string>
#include <atomic>

namespace sat {
namespace ui {

class IO {

public: 

   enum class Key {
      q = 'q',
      a = 'a',
      s = 's',
      enter = '\n'
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
   Key read();
};

}
}