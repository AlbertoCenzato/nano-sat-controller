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
GNU License V2 for more details: https://www.gnu.org/licenses/gpl-2.0.html

//==================================================================== */

#include "test/Test.hpp"

#include "NanoSat.hpp"
#include "devices/Interfaces.hpp"
#include "devices/IMU10DOF.hpp"
#include "devices/CurrentSensor.hpp"
#include "devices/MotorActuator.hpp"
#include "devices/FeedbackCoil.hpp"
#include "devices/AlignmentSensor.hpp"
#include "devices/PowerBoard.hpp"
#include "utils/Logger.hpp"
#include "utils/UI.hpp"
#include "utils/Settings.hpp"
#include "ctrl/Controller.hpp"

using std::this_thread::sleep_for;

using sat::utils::Logger;
namespace ui = sat::utils::ui;
using sat::device::ISensor;
using sat::device::IActuator;
using sat::device::IMU10DOF;

namespace sat {
	
namespace test {
	
const string Test::brk = string(100, '-');

// ---------- constructors ----------
#pragma region constructors

Test::Test(const NanoSat* satellite) : satellite_(satellite) { }

#pragma endregion constructors


// ---------- public member functions ----------
#pragma region publ_functions

void Test::currentSensor() const {

	vector<const ISensor*> sensors;
	auto sensor = satellite_->getCurrentSensor(Axis::X);
	if (sensor->isAvailable())
		sensors.push_back(sensor);
	sensor = satellite_->getCurrentSensor(Axis::Y);
	if (sensor->isAvailable())
		sensors.push_back(sensor); 
	sensor = satellite_->getCurrentSensor(Axis::Z);
	if (sensor->isAvailable())
		sensors.push_back(sensor);

	readLoop(sensors, header("Current sensor test"));
}

void Test::dcMotor() const {

	string hdr = header("DC motor test");
	Log::info << hdr;

	vector<IActuator*> motors;
	auto actuator = satellite_->getDCMotor(Axis::X);
	if (actuator->isAvailable())
		motors.push_back(actuator);

	actuator = satellite_->getDCMotor(Axis::Y);
	if (actuator->isAvailable())
		motors.push_back(actuator);

	actuator = satellite_->getDCMotor(Axis::Z);
	if (actuator->isAvailable())
		motors.push_back(actuator);

	for (auto motor : motors) {

		Log::info << motor->getID();
		Log::info << "Forward! ";

		Log::info << "\tSpeed up...";
		for (int i = 0; i < 255; ++i) {
			motor->act(i);
			sleep_for(chrono::milliseconds(10));
		}

		Log::info << "\tSlow down...";
		for (int i = 255; i >= 0; --i) {
			motor->act(i);
			sleep_for(chrono::milliseconds(10));
		}

		Log::info << "Backward! ";

      Log::info << "\tSpeed up...";
		for (int i = 0; i > -255; --i) {
			motor->act(i);
			sleep_for(chrono::milliseconds(10));
		}

      Log::info << "\tSlow down...";
		for (int i = -255; i <= 0; ++i) {
			motor->act(i);
			sleep_for(chrono::milliseconds(10));
		}

      Log::info << "Release";
		motor->act(0);
		sleep_for(chrono::seconds(1));
	}
}

void Test::testLogger() const {
   cout << header("Logger test") << endl;
	try {
      cout << "Getting logger..." << endl;
		cout << "Done!" << endl;
      cout << "Writing with info priority..." << endl;
		Log::info << "Prova di log";
		cout << "Done!" << endl;
	}
	catch (const spdlog::spdlog_ex& ex) {
		std::cerr << "Logger construction failed! " << ex.what() << std::endl;
	}

   ui::waitKey();
}

void Test::imu10DOF() const {
	string hdr = header("IMU 10 DOF test");

	auto imu = satellite_->getIMU();
	if (!imu->isAvailable()) {
		Log::info << "IMU unavailable";
		return;
	}

	const auto start = chrono::high_resolution_clock::now();
	auto currTime = chrono::high_resolution_clock::now();

	while (currTime < start + chrono::seconds(20)) {

		auto acc   = imu->readAccelerometer();
		auto gyr   = imu->readGyrosope();
		auto state = imu->getState();
		
		ui::clearConsole();
      cout << hdr << endl;

      cout << "Accelerometer:             " << acc   << endl;
		cout << "Gyroscope:                 " << gyr   << endl;
		cout << "State (X,Y,Z):             " << state << endl;

		sleep_for(chrono::milliseconds(50));

		currTime = chrono::high_resolution_clock::now();
	}
}


utils::Vector3f Test::getAngle(device::IMU10DOF* imu) const {
	const auto start = chrono::high_resolution_clock::now();
   utils::Vector3f angles{ 0.f, 0.f, 0.f };
	auto prevTime = chrono::high_resolution_clock::now();
	auto currTime = chrono::high_resolution_clock::now();
   //imu->startReading();
	auto prevValues = imu->readGyrosope();
	long count = 0;
	while (currTime < start + chrono::seconds(20)) {
		
		++count;

		currTime = chrono::high_resolution_clock::now();
		auto currValues = imu->readGyrosope();

		float dt = chrono::duration_cast<chrono::microseconds>(currTime - prevTime).count() / 10E6f;
      auto area = (prevValues + currValues)*(dt / 2);
      angles += area;

		ui::clearConsole();
		cout << "Read values: " << currValues << endl;
		cout << "Angles:      " << angles     << endl;

		prevTime = currTime;
		prevValues = currValues;
		sleep_for(chrono::milliseconds(5));
	}

	cout << "Misure effettuate: " << count << endl;

	return angles;
}


void Test::accelerometer() const {

	auto imu = satellite_->getIMU();
	if (!imu->getAccelerometer()->isAvailable()) {
		Log::info << "Accelerometer unavailable";
		return;
	}

	while (true) {

		auto value = imu->readAccelerometer();

		ui::clearConsole();
		cout << value << endl;

		sleep_for(chrono::milliseconds(50));	// refresh screen at 20fps
	}
}


void Test::gyroscopeAngles() const {
	
	string hdr = header("Static angles test");

	auto imu = satellite_->getIMU();
	if (!imu->getGyroscope()->isAvailable()) {
		Log::info << "Gyroscope unavailable";
		return;
	}
	
	auto angles = getAngle(imu);
   auto gains = angles / 90;

	cout << "Gains: " << gains << endl;
}


void Test::gyroscopeOffsets() const {

	string hdr = header("Offset angles test");

	auto imu = satellite_->getIMU();
	if (!imu->getGyroscope()->isAvailable()) {
		Log::info << "Gyroscope unavailable";
		return;
	}

	utils::Vector3f mean;
	long reads = 0;

	const auto start = chrono::high_resolution_clock::now();
	auto now = chrono::high_resolution_clock::now();
	while(now < start + chrono::seconds(20)) {
		mean += imu->readGyrosope();
		++reads;
	}

	mean = mean / reads;
	cout << "Mean: " << mean << endl;
}

void Test::magnetometer() const {
	
	auto imu = satellite_->getIMU();
	if (!imu->getMagnetometer()->isAvailable()) {
		Log::info << "Magnetometer unavailable";
		return;
	}

	const auto start = chrono::high_resolution_clock::now();
	auto currTime = chrono::high_resolution_clock::now();
	while (currTime < start + chrono::seconds(15)) {

      utils::Vector3f mean;
      for (auto i = 0; i < 10; ++i) {
         mean += imu->readMagnetometer();
      }
      cout << "(X, Y, Z): " << mean / 10 << endl;
		
	   sleep_for(chrono::milliseconds(50));
      ui::clearConsole();
      currTime = chrono::high_resolution_clock::now();
	}
}


void Test::coil() const {
	
	auto hdr = header("Coil test");
	Log::info << hdr;

	vector<pair<IActuator*, ISensor*>> actSens;
	actSens.emplace_back(satellite_->getCoil(Axis::X), satellite_->getCurrentSensor(Axis::X));
	actSens.emplace_back(satellite_->getCoil(Axis::Y), satellite_->getCurrentSensor(Axis::Y));
	actSens.emplace_back(satellite_->getCoil(Axis::Z), satellite_->getCurrentSensor(Axis::Z));
	
	for (auto &pair : actSens) {
		Log::info << pair.first->getID();
		for (int i = 0; i < 255; ++i) {
			pair.first->act(254);
			sleep_for(chrono::milliseconds(10));

			auto value = readMeanFor(pair.second, chrono::seconds(3));
			ui::clearConsole();
			ui::print(value);
		}

		Log::info << "Release";
		pair.first->act(0);
		sleep_for(chrono::seconds(1));
	}
}

void Test::feedbackFreq() const {

	auto coil = satellite_->getCoil(Axis::X);
	const auto start = chrono::high_resolution_clock::now();
	const int iterations = 1000;
	for (auto i = 0; i < iterations; ++i) {
		coil->act(50);
	}
	auto now = chrono::high_resolution_clock::now();

	auto elapsedTime = chrono::duration_cast<chrono::milliseconds>(now - start);
	cout << "Elapsed time: " << elapsedTime.count() << "ms" << endl;
	auto singleCallTime = elapsedTime.count() / float(iterations);
	cout << "Time per call: " << singleCallTime << "ms" << endl;
	cout << "Call frequency: " << 1 / (singleCallTime / 1000) << "Hz" << endl;

	coil->act(0);
}

void Test::camera() const {
	
	utils::AlignmentSensorSettings settings;
	device::AlignmentSensor camera(settings);
	camera.readCamera();
}

void Test::readSpeed() const {
	
	auto sensor = satellite_->getCurrentSensor(Axis::X);
	const auto start = chrono::high_resolution_clock::now();
	const int iterations = 1000;
	for (auto i = 0; i < iterations; ++i) {
		sensor->readCurrent();
	}
	auto now = chrono::high_resolution_clock::now();

	auto elapsedTime = chrono::duration_cast<chrono::milliseconds>(now - start);
	cout << "Elapsed time: " << elapsedTime.count() << "ms" << endl;
	auto singleCallTime = elapsedTime.count() / float(iterations);
	cout << "Time per call: " << singleCallTime << "ms" << endl;
	cout << "Call frequency: " << 1 / (singleCallTime / 1000) << "Hz" << endl;
}

void Test::motorAxes() const {
	for (auto i = 0; i < 3; ++i) {
		string ax;
		switch(i) {
		case 0: ax = "X"; break;
		case 1: ax = "Y"; break;
		case 2: ax = "Z"; break;
		}
		cout << "Rotating motor on " << ax << " axis" << endl;
		auto motor = satellite_->getDCMotor(static_cast<Axis>(i));
		motor->act(20);
		sleep_for(chrono::seconds(10));
		motor->act(0);
	}
}


void Test::testGyroscopeImuError() const {
   
   try {
      ctrl::setMaxSchedulingPriority();
   }
   catch (const utils::scheduling_error &ex) {
      std::cerr << ex.what() << std::endl;
      return;
   }

   auto imu = satellite_->getIMU();

   const auto start = chrono::high_resolution_clock::now();
   auto prevTime = chrono::high_resolution_clock::now();
   auto currTime = chrono::high_resolution_clock::now();
   auto prevValues = imu->readGyrosope();
   auto angles = imu->getState();
   long count = 0;
   utils::Vector3f error{ 0.f, 0.f, 0.f };
   while (currTime < start + chrono::seconds(30)) {

      ++count;

      currTime = chrono::high_resolution_clock::now();
      auto gyroValues = imu->readGyrosope();
      auto imuValues  = imu->getState();

      // integrate using trapezioidal rule
      auto dt = chrono::duration_cast<chrono::microseconds>(currTime - prevTime).count() / 10E6f;
      auto dArea = (prevValues + gyroValues)* (dt / 2);
      angles += dArea;
      for (auto &angle : angles) {
         if (angle >  180) angle -= 360;
         if (angle < -180) angle += 360;
      }

      error += abs(angles - imuValues);

      ui::clearConsole();
      cout << "Gyroscope: " << angles    << endl;
      cout << "IMU:       " << imuValues << endl;

      prevTime = currTime;
      prevValues = gyroValues;
      sleep_for(chrono::milliseconds(50));
   }

   cout << "Mean error: " << error / count << endl;
}

void Test::testIMUTracking() const {
   cout << "Move the nano satellite to desired initial position." << std::endl;
   ui::waitKey("Press enter when ready to move. Then press enter again when in position.");
   sat::trackMotionUntilKeyPress(*satellite_, "Press enter when in position.");
   cout << "Initial position reached!" << endl;
   sleep_for(chrono::seconds(3));

   auto imu = satellite_->getIMU();
   const auto start = chrono::high_resolution_clock::now();
   auto currTime = chrono::high_resolution_clock::now();
   while (currTime < start + chrono::seconds(15)) {
      const auto state = imu->getState();
      ui::clearConsole();
      cout << "(X, Y, Z): " << state << endl;
      currTime = std::chrono::high_resolution_clock::now();
   }
}

void Test::powerBoard() const {
   /*
   gnublin_i2c bus;
   utils::PowerBoardSettings settings;
   settings.address = 0x0B;
   device::PowerBoard pwBoard(&bus, settings);

   std::cout << "ApiVersion: " << pwBoard.getApiVersion() << std::endl;
   auto pair = pwBoard.getFirmwareVersion();
   std::cout << "FirmwareVersion: " << pair.first << "." << pair.second << std::endl;
   std::cout << "PowerOnDelay: " << pwBoard.getPowerOnDelay() << std::endl;
   std::cout << "SerialNumber: " << pwBoard.getSerialNumber() << std::endl;
   std::cout << "ShutdownDelay: " << pwBoard.getShutdownDelay() << std::endl;
   std::cout << "Status: " << pwBoard.getStatus().StatusDetail() << std::endl;
   std::cout << "Voltage: " << pwBoard.getVoltage() << std::endl;
   std::cout << "Read (voltage): ";
   ui::print(pwBoard.read());
   */
   
   const auto pwBoard = satellite_->getPowerBoard();
   cout << "ApiVersion: " << pwBoard->getApiVersion() << endl;
   auto pair = pwBoard->getFirmwareVersion();
   cout << "FirmwareVersion: " << pair.first << "." << pair.second << endl;
   cout << "PowerOnDelay: " << pwBoard->getPowerOnDelay() << endl;
   cout << "SerialNumber: " << pwBoard->getSerialNumber() << endl;
   cout << "ShutdownDelay: " << pwBoard->getShutdownDelay() << endl;
   cout << "Status: " << pwBoard->getStatus().StatusDetail() << endl;
   cout << "Voltage: " << pwBoard->getVoltage() << endl;
   cout << "Read (voltage): ";
   ui::print(pwBoard->read());
   
}

void Test::availability() const {
	auto sensors = satellite_->getSensorsList();
	for (auto sensor : sensors) {
		Log::info << sensor->getID();
		if (sensor->isAvailable())
         Log::info << "Available";
		else
         Log::info << "Not available";
	}

	auto imu = satellite_->getIMU();
	auto accel = imu->getAccelerometer();
   Log::info << accel->getID();
	if (accel->isAvailable())
      Log::info << "Available";
	else
      Log::info << "Not available";

	auto gyros = imu->getGyroscope();
   Log::info << gyros->getID();
	if (gyros->isAvailable())
      Log::info << "Available";
	else
      Log::info << "Not available";

	auto magne = imu->getMagnetometer();
   Log::info << magne->getID();
	if (magne->isAvailable())
      Log::info << "Available";
	else
      Log::info << "Not available";

	auto press = imu->getPressureSensor();
   Log::info << press->getID();
	if (press->isAvailable())
      Log::info << "Available";
	else
      Log::info << "Not available";
}

#pragma endregion publ_functions

// ---------- private member functions ----------
#pragma region priv_functions

string Test::header(const string& str) const {
	const std::size_t SIZE = str.size();
	string head(SIZE, '-');
	return head + "\n" + str + "\n" + head + "\n";
}

void Test::readLoop(const ISensor* sensor, const string& header) const{
	vector<const ISensor*> sensors = {sensor};
	readLoop(sensors, header);
}

void Test::readLoop(const vector<const ISensor*>& sensors, const string& header) const {
	const auto SIZE = sensors.size();
	vector<string> output(SIZE);
	while (true) {

		for (std::size_t i = 0; i < SIZE; ++i) {
			output[i] = sensors[i]->toString();
		}

		ui::clearConsole();
		cout << header << endl;
		for (auto& str : output) {
         Log::info << str;
         Log::info << brk;
		}

		sleep_for(chrono::milliseconds(50));	// refresh screen at 20fps
	}
}

#pragma endregion priv_functions

}	// end test
}	// end sat