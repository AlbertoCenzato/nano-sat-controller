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

#include "devices/FeedbackCoil.hpp"

#include "devices/CurrentSensor.hpp"

namespace sat {
namespace device {

FeedbackCoil::FeedbackCoil() : MotorActuator(), feedbackSensor(nullptr) { }

FeedbackCoil::FeedbackCoil(const utils::MotorSettings &settings, const CurrentSensor *feedbackSensor)
   : MotorActuator(settings), feedbackSensor(feedbackSensor), pid(1/255) { }


void FeedbackCoil::act(float target) {

	auto lastState = feedbackSensor->readCurrent();
	for (auto i = 0; i < LOW_HIGH_LEVEL_CTRL_FREQ_RATIO; ++i) {
		const auto state = feedbackSensor->readCurrent();
		const auto newTarget = pid(state, target) + lastState;
		MotorActuator::act(newTarget);
		lastState = state;
	}
}

std::unique_ptr<FeedbackCoil> FeedbackCoil::create(const utils::MotorSettings & settings, const CurrentSensor * feedbackSensor) {
	return std::make_unique<FeedbackCoil>(settings, feedbackSensor);
}


} // namespace device
} // namespace sat