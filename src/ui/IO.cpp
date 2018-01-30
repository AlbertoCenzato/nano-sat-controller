#include "ui/IO.hpp"

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

namespace sat {
namespace ui {

template<>
Key IO::read() {

   struct termios old_tio;
   
   // get the terminal settings for stdin
   getattr(STDIN_FILENO, &old_tio);
   
   // we want to keep the old setting to restore them a the end
   to new_tio = old_tio;
   
   // disable canonical mode (buffered i/o) and local echo
   w_tio.c_lflag &=(~ICANON & ~ECHO);
   
   // set the new settings immediately
   setattr(STDIN_FILENO, TCSANOW, &new_tio);
   
   to c = getchar();
   intf("%d ",c);
   const auto key = Key(c);
   
   // restore the former settings
   setattr(STDIN_FILENO, TCSANOW, &old_tio);
   
   return key;      
}

void IO::waitKey(std::string message) {
   cout << message << endl;
   std::cin.sync();
   std::cin.get();
}

void IO::asyncWaitKey(std::atomic_bool &flag, std::string message) {
   std::thread waitingThread([&flag, message] () {
      waitKey(message);
      flag.store(true);
   });
   waitingThread.detach();
}

}
}