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

#include <opencv2/videoio.hpp>

#include "devices/Device.hpp"
#include "devices/Interfaces.hpp"
#include "self_test/TestResult.hpp"

namespace sat {

namespace utils { struct AlignmentSensorSettings; }
	
namespace device {


/**
 * @brief This class provides a measure of the satellite offset
 *		  wrt a fixed target. It uses the RPi camera to estimate
 *		  the alignment.
 *		 
 * @note All AlignmentSensor object share a unique pointer to 
 *		 the static camera capture object used for reading camera frames.
 *		 Be careful if you modify the static object!
 * @note Of course this class is not thread-safe
 */
class AlignmentSensor : public Device, public ISensor {

public:
	explicit AlignmentSensor(const utils::AlignmentSensorSettings& settings);
	virtual ~AlignmentSensor();

	std::vector<double> read() const override;

	void readCamera();

	static std::unique_ptr<AlignmentSensor> create(const utils::AlignmentSensorSettings& settings);

	TestResult testConnection() override;

private:

	// WARNING: because of the static nature of videoCap multiple modifications to 
	//			camera settings can cause problems, watch out!
	static std::unique_ptr<cv::VideoCapture> videoCap;
	static int alignmentSensorObjects;
};

using AlignmentSensorPtr = std::unique_ptr<AlignmentSensor>;

} // namespace device
} // namespace sat