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

#include "devices/AlignmentSensor.hpp"

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "utils/Settings.hpp"


namespace sat {
namespace device {


std::unique_ptr<cv::VideoCapture> AlignmentSensor::videoCap = nullptr;
int AlignmentSensor::alignmentSensorObjects = 0;


AlignmentSensor::AlignmentSensor(const utils::AlignmentSensorSettings& settings) : Device(settings.deviceID) {
	
	// in this way multiple instances of AlignmentSensor can coexist
	// sharing a unique VideoCapture object initialized the first time
	// that an AlignmentSensor is initialized. This static object is stored
	// in a unique_ptr because there's no guarantee about when a static 
	// object is initialized. The standard says "before usage", ok but when?
	// Better a lazy contructor than an unnecessarly locked camera
	if (videoCap == nullptr) {
		videoCap = std::make_unique<cv::VideoCapture>(0);
	   Log::debug << "Acquired camera.";
	}

	if (!videoCap->isOpened()) {
		Log::err << "Error! Unable to open video stream!";
		available = false;
	}

	++alignmentSensorObjects;
}

AlignmentSensor::~AlignmentSensor() {
	--alignmentSensorObjects;
	if (alignmentSensorObjects == 0) {	// if no more AlignmentSensor objects release the camera 
		videoCap.reset(nullptr);		   // otherwise it will be owned until program termination.
		Log::debug << "Released camera.";
	}
}

std::vector<double> AlignmentSensor::read() const {
	return std::vector<double>();
}

void AlignmentSensor::readCamera() {

	if (!available)
		return;

	cv::Mat picture;
	std::cout << "Reading image from camera..." << std::endl;
	videoCap->read(picture);	// TODO: grab directly in grayscale, see: https://stackoverflow.com/questions/11159506/opencv-videocapture-set-greyscale
	std::cout << "Done!" << std::endl;

	if (picture.empty()) {
		std::cout << "Empty image!" << std::endl;
		return;
	}

	cv::imwrite("/home/pi/Desktop/image.jpg", picture);

	cv::cvtColor(picture, picture, cv::COLOR_BGR2GRAY);

	cv::Canny(picture, picture, 50, 150, 3); // FIXME: magic numbers!
	
	cv::Mat lines;
	cv::HoughLines(picture, lines, 1, PI / 180, 200); // FIXME: magic numbers!

	for (auto i = 0; i < lines.rows; ++i) {
		std::cout << "Rho: " << lines.at<float>(i,0) << ";  Theta: " << lines.at<float>(i, 1) << std::endl;
	}

	cv::imwrite("/home/pi/Desktop/processed_image.jpg", picture);
}

std::unique_ptr<AlignmentSensor> AlignmentSensor::create(const utils::AlignmentSensorSettings & settings) {
	return std::make_unique<AlignmentSensor>(settings);
}

TestResult AlignmentSensor::testConnection() {
	return TestResult();
}


} // namespace device 
} // namespace sat