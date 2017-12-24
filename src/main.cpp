#include <iostream>
#include <chrono>

#include "config.h"
#include "NanoSat.hpp"
#include "test/Test.hpp"
#include "utils/Logger.hpp"
#include "utils/UI.hpp"
#include "utils/Settings.hpp"
#include "utils/OSSignalHandler.hpp"
#include "test/Calibration.hpp"

namespace test = sat::test;
using std::this_thread::sleep_for;

using namespace sat::utils;

void runTests(const sat::NanoSat& nanoSat);
void calibrate(sat::NanoSat& nanoSat);
void executeMainProgram(sat::NanoSat& satellite);


int main()
{
	try
	{
		OSSignalHandler::registerSignalHandlers();
	   
		// print startup message
		cout << "---------------------------------" << endl;
		cout << "| Nano Satellite Controller v" << sat::config::getVersion() << " |" << endl;
		cout << "---------------------------------\n\n" << endl;

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

      sleep_for(chrono::seconds(2));

		bool exit = false;
		while (!exit) {	// main program loop

         ui::clearConsole();	// clear screen

			// let the user choose operating mode
			int mode = ui::selectFromList({"run standard mode",
										          "run tests",
										          "calibrate",
			                               "exit"});
			// run in the selected mode
			switch (mode) {
			case 2:	runTests(satellite); break;
			case 3:	calibrate(satellite); break;
         case 4:  break;
			default: executeMainProgram(satellite);
			}
			
         exit = ui::yesNoAnswer("Exit program?");
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


void executeMainProgram(sat::NanoSat &satellite) {
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

   std::cout << "Move the nano satellite to desired initial position." << std::endl;
   sat::trackMotionUntilKeyPress(satellite, "Press enter when in position.");

	if (satellite.performHoming())
		std::cout << "Home reached!" << std::endl;
	else
		std::cout << "Couldn't reach target!" << std::endl;

	// do some maneuvers here

}


void runTests(const sat::NanoSat& nanoSat) {

   test::Test test(&nanoSat);

	std::string header("-------- TEST MODE ---------\n");

	// let the user choose test
	bool exit = false;
	while (!exit) {
      ui::clearConsole();
      cout << header << endl;

		auto mode = ui::selectFromList({"test IMU",
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
		                        "test controller (offline)",
		                        "gyroscope-imu comparison",
		                        "test imu tracking",
		                        "test power board"});

		switch (mode) {
		case  1: test.imu10DOF();		         break;
		case  2: test.dcMotor();		         break;
		case  3: test.currentSensor();	      break;
		case  4: test.gyroscopeAngles();       break;
		case  5: test.accelerometer();	      break;
		case  6: test.availability();	         break;
		case  7: test.magnetometer();	         break;
		case  8: test.coil();			         break;
		case  9: test.feedbackFreq();	         break;
		case 10: test.readSpeed();	            break;
		case 11: test.gyroscopeOffsets();      break;
		case 12: test.camera();			         break;
		case 13: test.motorAxes();		         break;
      case 14: test.testController();        break;
      case 16: test.testGyroscopeImuError(); break;
      case 17: test.testIMUTracking();       break;
      case 18: test.powerBoard();            break;
		default: break;
		}

      exit = ui::yesNoAnswer("Exit test mode?");
	}

}

void calibrate(sat::NanoSat &nanoSat) {

   test::Calibration calib(&nanoSat);
   std::string header("-------- CALIBRATION MODE ---------\n");

   bool exit = false;
   while (!exit) {
      ui::clearConsole();
      cout << header << endl;

      auto selection = ui::selectFromList({ "Calibrate gyroscope gains",
                                       "Calibrate magnetometer offsets" });
      switch (selection) {
      case 1: calib.calibrateGyroscopeGains();        break;
      case 2: calib.calibrateMagnetometerOffsets();   break;
      default: break;
      }

      exit = ui::yesNoAnswer("Exit calibration mode?");
   }
}