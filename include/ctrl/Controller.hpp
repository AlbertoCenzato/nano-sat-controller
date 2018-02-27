/*====================================================================
Nano satellite controller

// Copyright: Copyright (c) 2017, Alberto Cenzato
				  All rights reserved.

// Licence: GNU

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU License.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU License V3 for more details: https://www.gnu.org/licenses/gpl-3.0.html

//==================================================================== */

#pragma once


#include <memory>
#include <vector>
#include <ios>

#include "utils/DataTypes.hpp"
#include "ctrl/ControlAndFilter.hpp"
#include "utils/Exceptions.hpp"
#include "utils/Logger.hpp"
#include "utils/Settings.hpp"
#include "devices/IMU10DOF.hpp"
#include "utils/Vector.hpp"
#include "self_test/TestResult.hpp"


namespace sat {

namespace device { class IActuator; }


using device::IActuator;
using device::IMU10DOF;

namespace ctrl {

/**
 * @brief Sets current thread to maximum priority
 * @throws sat::utils::scheduling_error
 */
void setMaxSchedulingPriority();

/**
 *	@brief Restores default scheduling priority for current thread
 *	@throws sat::utils::scheduling_error
 */
void restoreDefaultPriority();


/**
 *	@brief Operation defines what state the nano sat should reach 
 *			 and what actuators should it use to reach it
 */
struct Operation {
	enum class Type {
		move,
		nop
	};

	Type type;
	IActuator *actuatorX;
	IActuator *actuatorY;
   IActuator *actuatorZ;
	IMU10DOF *imu;

	utils::Vector3f targetState;

   std::string toString() const noexcept;
};




/**
 * @brief This class is used by NanoSat to sequentially execute all the operations
 *		    it is given. 
 * @note If possible curent thread is executed as a real-time high priority
 *		   thread when executing the control loop.
 */
class Controller {
public:

	explicit Controller(const utils::ControllerSettings& settings = utils::ControllerSettings());
	~Controller() = default;


	/**
	 * @brief Adds an operation to the list of maneuvers to execute
	 * @param op: operation to do. It is required all op's members to be not null
	 * @return a reference to this Controller object so multiple operations can be added
	 *         (e.g. controller.addOperation(op1).addOperation(op2).addOperation(op3).run();)
	 */
	Controller& addOperation(Operation op);

	/**
	 * @brief Removes all operations from operation list
	 */
	void clearOperations();

	/**
	 *	@brief Executes sequentially all the operations added to this controller
	 *			 (with FIFO policy). The controller uses actuators specified by the operation
	 *			 to reach the final state. To compare current state with final state 
	 *			 controller's tolerance is used.
	 *			 When all operations are completed successfully operation list is cleared.
	 *	@return true if no error occurs
	 */
	bool run();

	/**
	 *	@brief Specifies the control function to be used (default control function is a PID)
	 */
	void setControlFunction(std::unique_ptr<ControlAlgorithm<utils::Vector3f>>&& controlFunction);

private:

	/**
	* @brief This class is used to replace Operation's missing actuators,
	*        letting Controller::run() working with one, two or three axis.
	*/
	class StubActuator : public device::IActuator {
	public:
		std::string getID() const noexcept override { return "Stub_Actuator"; }
		std::string toString() const noexcept override { return getID(); }
		TestResult selfTest() noexcept override { return {}; }
		bool isAvailable() const noexcept override { return false; }
		void act(float action) override {}
	};

	/**
	 *	@brief This struct represents the logic state Controller is in
	 */
	struct ControllerLogic {

		ControllerLogic(float tolerance, std::chrono::milliseconds ctrlLoopTimeout, std::chrono::milliseconds totCtrlTimeout);

		void reset(const utils::Vector3f &newFinalState, const std::array<bool, 3> &usedAxis);

		bool hasFinishedOperation(const utils::Vector3f &state);

	private:
		std::chrono::time_point<std::chrono::high_resolution_clock> startTime, reachingTime;
		utils::Vector3f target;
		bool targetReached = false;
		bool finishedOperation = false;
		std::array<bool, 3> usedAxis;
		
		const float tolerance;

		// when target reached controller continues running for CTRL_LOOP_TIMEOUT ms
		const std::chrono::milliseconds CTRL_LOOP_TIMEOUT;

		// if has run for more than TOT_CTRL_TIMEOUT ms on a single operation, 
		//	stop and continue with the next one
		const std::chrono::milliseconds TOT_CTRL_TIMEOUT;

		
		bool hasReachedTargetState(const utils::Vector3f &state);
	};

	const float tolerance;
	std::vector<Operation> opList;
	bool controlThreadReturnVal;
	std::unique_ptr<ControlAlgorithm<utils::Vector3f>> controlFunction;
   const uint32_t MEASUREMENTS_PER_CONTROL;

	ControllerLogic controllerState;
   mutable StubActuator stubActuator; // stub actuator used to replace operation's nullptrs

	/**
	 *	@brief checks if the given operation is properly defined;
	 *			 i.e. it has all actuators necessary to get to the final state.
	 *			 Called by run() before passing the operation to the controller.
	 *	@param op: operation to check.
	 */
	bool validateOperation(ctrl::Operation& op) const;

   /**
    * @brief private version of run(), it executes the control loop
    * TODO: this function should be noexcept. 
    * See function definition for what is missing to be noexcept
    */
	void run_();

	bool increaseSchedulingPriority() const;

	bool restoreSchedulingPriority() const;

	

};

} // namespace ctrl
} // namespace sat