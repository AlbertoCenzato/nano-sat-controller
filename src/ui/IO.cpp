#include "ui/IO.hpp"

#include <iostream>
#include <thread>

namespace sat {
namespace ui {

void waitKey(std::string message) {
	std::cout << message << std::endl;
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

}
}
