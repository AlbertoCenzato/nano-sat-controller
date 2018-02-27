#include <string>
#include <iostream>
#include <thread>
#include <chrono>

#include "config.h"
#include "NanoSat.hpp"
#include "test/Test.hpp"
#include "ui/UI.hpp"
#include "utils/Settings.hpp"
#include "utils/OSSignalHandler.hpp"
#include "test/Calibration.hpp"
#include "utils/Exceptions.hpp"
#include "../include/ui/UI.hpp"


namespace test = sat::test;
using std::this_thread::sleep_for;

using namespace sat::utils;

void runTests(const sat::NanoSat& nanoSat);
void calibrate(sat::NanoSat& nanoSat);
void executeMainProgram(sat::NanoSat& satellite, const Vector3f &target);


int main()
{
	try
	{
		OSSignalHandler::registerSignalHandlers();
	   
		// print startup message
		cout << "-----------------------------------" << endl;
		cout << "| Nano Satellite Controller v" << sat::config::getVersion() << " |" << endl;
		cout << "-----------------------------------\n\n" << endl;

		// load configuration file
		cout << "Loading configuration file..." << endl;
		const Settings settings;
		cout << "Configuration loaded!" << endl;

		Log::init(settings.global.logFilePath);
		Log::setLevel(settings.global.logLevel);

		// setup NanoSat object
		cout << "Setting up NanoSat..." << endl;
		sat::NanoSat satellite(settings.nanoSat);
		cout << "Setup done!" << endl;

		sleep_for(std::chrono::seconds(2));

		bool exit = false;
		while (!exit) {	// main program loop

			sat::ui::clearConsole();	// clear screen

			// let the user choose operating mode
			const auto mode = sat::ui::selectFromList({ "run standard mode",
										                "run tests",
										                "calibrate",
														"exit"});
			// run in the selected mode
			switch (mode) {
			case 2:	runTests(satellite); break;
			case 3:	calibrate(satellite); break;
			case 4:  break;
			default: executeMainProgram(satellite, settings.global.target);
			}
			
         exit = sat::ui::yesNoAnswer("Exit program?");
		}

	}
	catch (const TerminateProgramException &ex) {
		std::cout << ex.what() << std::endl;
	}
	catch (const std::exception& ex) {
		std::cerr << "Exception thrown in main!\n" << ex.what() << std::endl;
	}
	catch (...)	{
		std::cerr << "Unknown exception thrown in main!!!" << std::endl;
	}

	return 0;
}


void executeMainProgram(sat::NanoSat &satellite, const Vector3f &target) {
	// ----- perform startup tests -----
   Log::info << "Performing tests for all attached devices...";
   auto result = satellite.selfTest();
	if (result.hasErrOrWarn()) {
		Log::err << "WARNING: Not all devices passed tests!";
		Log::err << result;
	}
	else {
		Log::info << "Tests passed!";
	}

	auto axis = sat::ui::multipleSelectFromList({ "use X axis", "use Y axis", "use Z axis" });

   std::cout << "Re-calibrating IMU... hold the nano sat in (0,0,0)... " << std::endl;
   satellite.getIMU()->reset();
   std::cout << "Done!" << std::endl;

   sat::trackMotionUntilKeyPress(satellite, 
      "Move the nano satellite to desired initial position. Press [ENTER] when in position.");

   sat::ctrl::Operation op;
	if (axis[0])
		op.actuatorX = satellite.getDCMotor(Axis::X);
	else
		op.actuatorX = nullptr;
	if (axis[1])
		op.actuatorY = satellite.getDCMotor(Axis::Y);
	else
		op.actuatorY = nullptr;
	if (axis[2])
		op.actuatorZ = satellite.getDCMotor(Axis::Z);
	else
		op.actuatorZ = nullptr;
   op.imu         = satellite.getIMU();
   op.targetState = target;

	if (satellite.move(op))
		std::cout << "Target reached!" << std::endl;
	else
		std::cout << "Couldn't reach target!" << std::endl;

	// do some operations here

}


void runTests(const sat::NanoSat& nanoSat) {

   test::Test test(&nanoSat);

	std::string header("-------- TEST MODE ---------\n");

	// let the user choose test
	bool exit = false;
	while (!exit) {
		sat::ui::clearConsole();
      cout << header << endl;

		auto mode = sat::ui::selectFromList({"test IMU",
											 "test DC motor",
											 "test current sensor",
											 "test gyroscope angles",
											 "test accelerometer", 
											 "test availability",
											 "test magnetometer", 
											 "test coils",
											 "test coil feedback frequency",
											 "test read speed",
											 "test gyroscope offsets",
											 "test camera",
											 "test motor-axes assignment",
											 "gyroscope-imu comparison",
											 "test imu tracking",
											 "test power board",
											 "back"});

		switch (mode) {
		case  1: test.imu10DOF();		     break;
		case  2: test.dcMotor();		     break;
		case  3: test.currentSensor();	     break;
		case  4: test.gyroscopeAngles();     break;
		case  5: test.accelerometer();	     break;
		case  6: test.availability();	     break;
		case  7: test.magnetometer();	     break;
		case  8: test.coil();			     break;
		case  9: test.feedbackFreq();	     break;
		case 10: test.readSpeed();	         break;
		case 11: test.gyroscopeOffsets();    break;
		case 12: test.camera();			     break;
		case 13: test.motorAxes();		     break;
		case 14: test.testGyroscopeImuError(); break;
		case 15: test.testIMUTracking();       break;
		case 16: test.powerBoard();            break;
		case 17: break;
		default: break;
		}

      exit = sat::ui::yesNoAnswer("Exit test mode?");
	}

}

void calibrate(sat::NanoSat &nanoSat) {

   test::Calibration calib(&nanoSat);
   const std::string header("-------- CALIBRATION MODE ---------\n");

   auto exit = false;
   while (!exit) {
	   sat::ui::clearConsole();
      cout << header << endl;

      const auto selection = sat::ui::selectFromList({ "Calibrate gyroscope gains",
                                                  "Calibrate magnetometer offsets",
                                                  "Back"});
      switch (selection) {
      case 1: calib.calibrateGyroscopeGains();        break;
      case 2: calib.calibrateMagnetometerOffsets();   break;
      case 3:  break;
      default: break;
      }

      exit = sat::ui::yesNoAnswer("Exit calibration mode?");
   }
}