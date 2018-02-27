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
#include "ui/UI.hpp"


using std::vector;
using sat::utils::Vector3f;
using sat::utils::scheduling_error;

int policy;
sched_param param;
cpu_set_t cpuSet;

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
      stream << "\nTarget state: " << targetState;
      str = stream.str();
   }
   catch (const std::exception &ex) {
      Log::err << ex.what();
   }
   return str;
}




// --------------------------------------------------------------
// ----------------------- ControllerLogic ----------------------
// --------------------------------------------------------------

Controller::ControllerLogic::ControllerLogic(float tolerance, std::chrono::milliseconds ctrlLoopTimeout, std::chrono::milliseconds totCtrlTimeout)
	: tolerance(tolerance), CTRL_LOOP_TIMEOUT(ctrlLoopTimeout), TOT_CTRL_TIMEOUT(totCtrlTimeout)
{
	usedAxis = { false, false, false };
}

void Controller::ControllerLogic::reset(const Vector3f &newFinalState, const std::array<bool, 3> &usedAxis) {
	target = newFinalState;
	targetReached = false;
	finishedOperation = false;
	this->usedAxis = usedAxis;
	startTime = std::chrono::high_resolution_clock::now();
}

bool Controller::ControllerLogic::hasFinishedOperation(const Vector3f &state) {
	if (finishedOperation)
		return true;

	// start control loop timeout if target reached
	if (!targetReached && hasReachedTargetState(state)) {
		reachingTime = std::chrono::high_resolution_clock::now();
		targetReached = true;
	}

	// exit if timeout elapsed
	if (targetReached && (reachingTime + CTRL_LOOP_TIMEOUT) < std::chrono::high_resolution_clock::now()) {
		finishedOperation = true;
		return true;
	}

	// exit if has run for more than TOT_CTRL_TIMEOUT milliseconds
	if ((startTime + TOT_CTRL_TIMEOUT) < std::chrono::high_resolution_clock::now()) {
		finishedOperation = true;
		return true;
	}

	return false;
}

bool Controller::ControllerLogic::hasReachedTargetState(const Vector3f &state) {
	for (auto i = 0; i < 3; ++i) {
		if (usedAxis[i]) {
			if (std::abs(state[i] - target[i]) > tolerance)
				return false;
		}
	}
	return true;
}




// --------------------------------------------------------------
// ----------------------- Controller ---------------------------
// --------------------------------------------------------------

Controller::Controller(const utils::ControllerSettings& settings) : tolerance(settings.tolerance), opList(0), controlThreadReturnVal(false),
   MEASUREMENTS_PER_CONTROL(settings.measurementsPerControl), controllerState(settings.tolerance, settings.ctrlLoopTimeout, settings.totCtrlTimeout)
{
	controlFunction = PID<Vector3f>::create(settings.kp, settings.ki, settings.kd);
}


Controller& Controller::addOperation(Operation op) {
	opList.push_back(op);
	return *this;
}


bool Controller::run() {

   for (auto &op : opList) {
      if (!validateOperation(op)) {
         ui::logAndPrint(Log::err, "INVALID OPERATION: " + op.toString());
			ui::logAndPrint(Log::err, "Operation will be skipped.");
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

void Controller::setControlFunction(std::unique_ptr<ControlAlgorithm<utils::Vector3f>>&& controlFunction) {
	this->controlFunction = std::move(controlFunction);
}


void Controller::run_() {

	increaseSchedulingPriority();

	// perform sequentially all operations in the list
	for (const auto& op : opList) {

		if (op.type == Operation::Type::nop) {
			ui::logAndPrint(Log::debug, op.toString() + "\n\nSkipping operation\n\n");
			continue;
		}
			
      assert(op.actuatorX != nullptr && op.actuatorY != nullptr 
         && op.actuatorZ != nullptr && op.imu != nullptr);

		ui::logAndPrint(Log::info, "Trying operation: " + op.toString());
		std::cout << "\nOperation will start in 3 seconds..." << std::endl;
		std::this_thread::sleep_for(std::chrono::seconds(3));

		try {
			std::array<bool, 3> usedAxis{ op.actuatorX->isAvailable(), op.actuatorY->isAvailable(), op.actuatorZ->isAvailable() };
			
		   auto state	  = op.imu->getState();
         auto velocity = op.imu->readGyrosope();

			Vector3f lastControlOutput{0.f,0.f,0.f};

         // ----- control loop -----
			controllerState.reset(op.targetState, usedAxis);
			while (!controllerState.hasFinishedOperation(state)) {	// check if targetState reached or timeout elapsed

				// compute feedback control
            const auto controlOutput = (*controlFunction)(state, velocity, op.targetState) + lastControlOutput;

				std::cout << "State: " << state << ";   Control output: " << controlOutput << std::endl;

            op.actuatorX->act(controlOutput[0]);
            op.actuatorY->act(controlOutput[1]);
            op.actuatorZ->act(controlOutput[2]);
				
				lastControlOutput = controlOutput;

            state.fill(0.f);
            velocity.fill(0.f);
            for (uint32_t i = 0; i < MEASUREMENTS_PER_CONTROL; ++i) {
               state    += op.imu->getState();
               velocity += op.imu->readGyrosope();
            }
            state    = state    / MEASUREMENTS_PER_CONTROL;
            velocity = velocity / MEASUREMENTS_PER_CONTROL;

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
			}
		}
		catch (const std::exception& ex) {
         op.actuatorX->act(0);   // WARNING: act isn't noexcept
         op.actuatorY->act(0);   // WARNING: act isn't noexcept
         op.actuatorZ->act(0);   // WARNING: act isn't noexcept

         ui::logAndPrint(Log::err, "OOPs! Something went wrong with current operation " + op.toString());
			controlThreadReturnVal = false;

			// restore this thread's default scheduling priority
			restoreSchedulingPriority();

			return;
		}

		op.actuatorX->act(0);
		op.actuatorY->act(0);
		op.actuatorZ->act(0);
	}

	restoreSchedulingPriority();

	controlThreadReturnVal = true;
}

bool Controller::increaseSchedulingPriority() const {
	auto success = false;
	try {
		setMaxSchedulingPriority();
		success = true;
	}
	catch (const scheduling_error& sched_err) {
		ui::logAndPrint(Log::err, "Error while increasing controller thread priority!");
		ui::logAndPrint(Log::err, sched_err.what());
	}

	return success;
}

bool Controller::restoreSchedulingPriority() const {
	auto success = false;
	try {
		restoreDefaultPriority();
		success = true;
	}
	catch (const scheduling_error& sched_err) {
		ui::logAndPrint(Log::err, "Error while increasing controller thread priority!");
		ui::logAndPrint(Log::err, sched_err.what());
	}

	return success;
}



void Controller::clearOperations() {
	opList.clear();
}



bool Controller::validateOperation(Operation& op) const {
   Log::debug << "Checking if valid operation...";

	if (op.imu == nullptr) {
		op.type = Operation::Type::nop;
      ui::logAndPrint(Log::err, "Undefined IMU!");
		return false;
	}

	auto state = op.imu->getState();

   int actuatorCount = 3;
   if (op.actuatorX == nullptr) {
		if (state[0] < op.targetState[0] - tolerance || state[0] > op.targetState[0] + tolerance)
			ui::logAndPrint(Log::warn,"You want me to move along X axis, but you did not declare an X axis actuator!");

      op.actuatorX = &stubActuator;
      --actuatorCount;
   }
   if (op.actuatorY == nullptr) {
      if (state[1] < op.targetState[1] - tolerance || state[1] > op.targetState[1] + tolerance)
			ui::logAndPrint(Log::warn, "You want me to move along Y axis, but you did not declare an Y axis actuator!");
      
      op.actuatorY = &stubActuator;
      --actuatorCount;
   }
   if (op.actuatorZ == nullptr) {
      if (state[2] < op.targetState[2] - tolerance || state[2] > op.targetState[2] + tolerance)
			ui::logAndPrint(Log::warn, "You want me to move along Z axis, but you did not declare an Z axis actuator!");

      op.actuatorZ = &stubActuator;
      --actuatorCount;
   }

   if (actuatorCount < 1) {
		op.type = Operation::Type::nop;
		ui::logAndPrint(Log::err, "No actuator defined! Empty operation!");
      return false;
   }

	op.type = Operation::Type::move;
	return true;
}


void setMaxSchedulingPriority() {
   const auto this_pthread = pthread_self();

	// save current scheduling and affinity params
	auto schedPolicyError = pthread_getschedparam(this_pthread, &policy, &param);
	switch (schedPolicyError) {
		case 0:		break; // everything fine
		case ESRCH: throw scheduling_error("No thread with the ID thread could be found.");
		default:		throw scheduling_error("Unknown error by pthread_getschedparam()");
	}

	auto affinityError = pthread_getaffinity_np(this_pthread, sizeof(cpu_set_t), &cpuSet);
	switch (affinityError) {
		case 0:		 break; // everything fine
		case EFAULT: throw scheduling_error("A supplied memory address was invalid.");
		case EINVAL: throw scheduling_error("EINVAL error. See pthread_setaffinity_np() documentation");
		case ESRCH:  throw scheduling_error("No thread with the ID thread could be found.");
		default:		 throw scheduling_error("Unknown error by pthread_getaffinity_np");
	}

	// set high priority scheduling params
	sched_param highPriorityParam;
	highPriorityParam.__sched_priority = sched_get_priority_max(SCHED_FIFO);
   schedPolicyError = pthread_setschedparam(this_pthread, SCHED_FIFO, &highPriorityParam);
   switch (schedPolicyError) {
		case 0:		  break; // everything fine
		case ESRCH:   throw scheduling_error("No thread with the ID thread could be found.");
		case EINVAL:  throw scheduling_error("Policy is not a recognized policy, or param does not make sense for the policy.");
		case EPERM:   throw scheduling_error("The caller does not have appropriate privileges to \
		                                      set the specified scheduling policy and parameters.\n \
		                                      Try running with sudo.");
		case ENOTSUP: throw scheduling_error("attempt was made to set the policy or scheduling parameters to an unsupported value");
		default:		  throw scheduling_error("Unknown error by pthread_setschedparam()");
   }

   // Create a cpu_set_t object representing a set of CPUs. Clear it and mark
   // only CPU i as set.
   cpu_set_t affinityCPUSet;
   CPU_ZERO(&affinityCPUSet);   // clear cpuset mask
   CPU_SET(sched_getcpu(), &affinityCPUSet); // set cpuset mask to current cpu
   affinityError = pthread_setaffinity_np(this_pthread, sizeof(cpu_set_t), &affinityCPUSet);
   switch(affinityError) {
		case 0: break; // everything fine
   	case EFAULT: throw scheduling_error("A supplied memory address was invalid.");
		case EINVAL: throw scheduling_error("EINVAL error. See pthread_setaffinity_np() documentation");
		case ESRCH:  throw scheduling_error("No thread with the ID thread could be found.");
		default:		 throw scheduling_error("Unknown error by pthread_setaffinity_np()");
   }
}

void restoreDefaultPriority() {
	const auto this_pthread = pthread_self();

	// set high priority scheduling params
	const auto schedPolicyError = pthread_setschedparam(this_pthread, policy, &param);
	switch (schedPolicyError) {
		case 0:		  break; // everything fine
		case ESRCH:   throw scheduling_error("No thread with the ID thread could be found.");
		case EINVAL:  throw scheduling_error("Policy is not a recognized policy, or param does not make sense for the policy.");
		case EPERM:   throw scheduling_error("The caller does not have appropriate privileges to \
			                                      set the specified scheduling policy and parameters.\n \
			                                      Try running with sudo.");
		case ENOTSUP: throw scheduling_error("attempt was made to set the policy or scheduling parameters to an unsupported value");
		default:		  throw scheduling_error("Unknown error by pthread_setschedparam()");
	}

	const auto affinityError = pthread_setaffinity_np(this_pthread, sizeof(cpu_set_t), &cpuSet);
	switch (affinityError) {
		case 0: break; // everything fine
		case EFAULT: throw scheduling_error("A supplied memory address was invalid.");
		case EINVAL: throw scheduling_error("EINVAL error. See pthread_setaffinity_np() documentation");
		case ESRCH:  throw scheduling_error("No thread with the ID thread could be found.");
		default:		 throw scheduling_error("Unknown error by pthread_setaffinity_np()");
	}
}


} // end ctrl

} // end sat

