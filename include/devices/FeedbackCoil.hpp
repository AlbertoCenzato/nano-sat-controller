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

#include "devices/MotorActuator.hpp"
#include "ctrl/ControlAndFilter.hpp"

namespace sat {

namespace device {

class CurrentSensor;


class FeedbackCoil : public MotorActuator {
	
public:
	FeedbackCoil();
	FeedbackCoil(const utils::MotorSettings &settings, const CurrentSensor *feedbackSensor);

	void act(float action) override;

	static std::unique_ptr<FeedbackCoil> create(const utils::MotorSettings& settings,
												const CurrentSensor *feedbackSensor);

protected:

	const int LOW_HIGH_LEVEL_CTRL_FREQ_RATIO = 10;
	const CurrentSensor *feedbackSensor;
	ctrl::PID<float> pid;

};


using FeedbackCoilPtr = std::unique_ptr<FeedbackCoil>;


} // namespace device
} // namespace sat