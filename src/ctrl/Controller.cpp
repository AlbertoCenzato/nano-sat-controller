/*====================================================================
Nano satellite controller

// Copyright   : Copyright (c) 2017, Alberto Cenzato
All rights reserved.

// Licence: GNU

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU License.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU License V3 for more details: https://www.gnu.org/licenses/gpl-3.0.html

//==================================================================== */

#include "ctrl/Controller.hpp"

#include <thread>
#include <sched.h>
#include <pthread.h>
#include <iostream>
#include <sstream>

#include "utils/Logger.hpp"
#include "devices/Interfaces.hpp"
#include "utils/Exceptions.hpp"
#include "utils/UI.hpp"


using std::vector;
using sat::utils::Vector3f;
using sat::utils::scheduling_error;


namespace sat {
namespace ctrl {

// --------------------------------------------------------------
// ---------------------- Operation -----------------------------
// --------------------------------------------------------------

std::string Operation::toString() const noexcept {
   std::string str;
   try {
      std::stringstream stream;
      stream << "X-axis actuator:\n" << actuatorX->toString();
      stream << "\nY-axis actuator:\n" << actuatorY->toString();
      stream << "\nZ-axis actuator:\n" << actuatorZ->toString();
      stream << "\nIMU:\n" << imu->toString();
      stream << "\nTarget state: " << finalState;
      str = stream.str();
   }
   catch (const std::exception &ex) {
      Log::err << ex.what();
   }
   return str;
}




// --------------------------------------------------------------
// ----------------------- Controller ---------------------------
// --------------------------------------------------------------

Controller::Controller(const utils::ControllerSettings& settings) 
	: tolerance(settings.tolerance), opList(0), controlThreadReturnVal(false) {

	controlFunction = PID<Vector3f>::create(settings.kp, settings.ki, settings.kd);
}


Controller& Controller::addOperation(Operation op) {
	opList.push_back(op);
	return *this;
}


bool Controller::run() {

   for (const auto &op : opList) {
      if (!checkIfValid(op)) {
         Log::err << "INVALID OPERATION: " << op.toString();
         return false;
      }
   }

   // TODO: run_ should be noexcept to safely terminate 
   //       and get control back to this thread
	//std::thread controlThread(&Controller::run_, this);   
	//if (controlThread.joinable())
	//	controlThread.join();
   run_();
   
	if (controlThreadReturnVal)
		opList.clear();

	return controlThreadReturnVal;
}

float Controller::getTolerance() const {
	return tolerance;
}

void Controller::setControlFunction(std::unique_ptr<ControlAlgorithm<utils::Vector3f>>&& controlFunction) {
	this->controlFunction = std::move(controlFunction);
}


void Controller::run_() {

   // try increasing this thread's priority
   try {
      setMaxSchedulingPriority();
   }
   catch (const scheduling_error& sched_err) {
      const std::string message("Error while increasing controller thread priority!");
      Log::err << message;
      Log::err << sched_err.what();
      std::cerr << message << std::endl;
      std::cerr << sched_err.what() << std::endl;
   }

	// perform sequentially all operations in the list
	for (const auto& op : opList) {

      assert(op.actuatorX != nullptr && op.actuatorY != nullptr 
         && op.actuatorZ != nullptr && op.imu != nullptr);

		Log::info << "Trying operation: " << op.toString();
      std::cout << "Trying operation: " << op.toString() << std::endl;

      // ask for user confirmation
		if (!utils::ui::yesNoAnswer("Go?")) {
			std::cout << "Aborting operation." << std::endl;
			controlThreadReturnVal = false;
			return;
		}

		try {
			auto imu = op.imu;
			const auto &finalState = op.finalState;
			auto actuatorX = op.actuatorX;
			auto actuatorY = op.actuatorY;
			auto actuatorZ = op.actuatorZ;
			auto state = imu->getState();
			Vector3f lastControlOutput{0.f,0.f,0.f};

         // execute control loop
			while (/*!equalsWithTolerance(state, finalState, tolerance)*/true) {

				// compute feedback control
            const auto controlOutput = (*controlFunction)(state, finalState) + lastControlOutput;

				std::cout << "State: " << state << ";   Control output: " << controlOutput << std::endl;

            actuatorX->act(controlOutput[0]);
            actuatorY->act(controlOutput[1]);
            actuatorZ->act(controlOutput[2]);
				
				lastControlOutput = controlOutput;

            state = { 0.f,0.f,0.f };
            for (auto i = 0; i < MEASUREMENTS_PER_CONTROL; ++i) {
               state += imu->getState();
            }
            state = state / MEASUREMENTS_PER_CONTROL;

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
			}
		}
		catch (const std::exception& ex) {
         op.actuatorX->act(0);   // WARNING: act isn't noexcept
         op.actuatorY->act(0);   // WARNING: act isn't noexcept
         op.actuatorZ->act(0);   // WARNING: act isn't noexcept

         Log::err << "OOPs! Something went wrong with current operation " << op.toString();
			controlThreadReturnVal = false;
			return;
		}

		op.actuatorX->act(0);   // TODO: act should be noexcept to enable run_() to be noexcept
		op.actuatorY->act(0);   // TODO: act should be noexcept to enable run_() to be noexcept
		op.actuatorZ->act(0);   // TODO: act should be noexcept to enable run_() to be noexcept
	}

	controlThreadReturnVal = true;
}

void Controller::clearOperations() {
	opList.clear();
}



bool Controller::checkIfValid(const Operation& op) const {
   Log::debug << "Checking if valid operation...";

	if (op.imu == nullptr) {
      Log::err << "Undefined IMU!";
		return false;
	}

	auto state = op.imu->getState();

   if (op.actuatorX == nullptr) {
      if (state[0] < op.finalState[0] - tolerance || state[0] > op.finalState[0] + tolerance) {
         Log::err << "You want me to move along X axis, but you did not declare an X axis actuator!";
         return false;
      }
      Log::err << "No X-axis actuator provided!";
      return false;
   }
   if (op.actuatorY == nullptr) {
      if (state[1] < op.finalState[1] - tolerance || state[1] > op.finalState[1] + tolerance) {
         Log::err << "You want me to move along Y axis, but you did not declare an Y axis actuator!";
         return false;
      }
      Log::err << "No Y-axis actuator provided!";
      return false;
   }
   if (op.actuatorZ == nullptr) {
      if (state[2] < op.finalState[2] - tolerance || state[2] > op.finalState[2] + tolerance) {
         Log::err << "You want me to move along Z axis, but you did not declare an Z axis actuator!";
         return false;
      }
      Log::err << "No Z-axis actuator provided!";
      return false;
   }
   Log::debug << "Done! Valid operation.";
	return true;
}


void setMaxSchedulingPriority() {
   // schedule this thread as real-time program with max priority
   const auto this_pthread = pthread_self();

   sched_param param;
   param.__sched_priority = sched_get_priority_max(SCHED_FIFO);
   const auto schedPolicyError = pthread_setschedparam(this_pthread, SCHED_FIFO, &param);
   switch (schedPolicyError) {
   case ESRCH:   throw scheduling_error("No thread with the ID thread could be found.");
   case EINVAL:  throw scheduling_error("Policy is not a recognized policy, or param does not make sense for the policy.");
   case EPERM:   throw scheduling_error("The caller does not have appropriate privileges to \
                                         set the specified scheduling policy and parameters.\n \
                                         Try running with sudo.");
   case ENOTSUP: throw scheduling_error("attempt was made to set the policy or scheduling parameters to an unsupported value");
   }

   // Create a cpu_set_t object representing a set of CPUs. Clear it and mark
   // only CPU i as set.
   cpu_set_t cpuset;
   CPU_ZERO(&cpuset);   // clear cpuset mask
   CPU_SET(sched_getcpu(), &cpuset); // set cpuset mask to current cpu
   auto affinityError = pthread_setaffinity_np(this_pthread, sizeof(cpu_set_t), &cpuset);
   switch(affinityError) {
   case EFAULT: throw scheduling_error("A supplied memory address was invalid.");
   case EINVAL: throw scheduling_error("EINVAL error. See pthread_setaffinity_np() documentation");
   case ESRCH:  throw scheduling_error("No thread with the ID thread could be found.");
   }
}


} // end ctrl

} // end sat

