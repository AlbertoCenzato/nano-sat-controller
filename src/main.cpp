#include <iostream>
#include <chrono>

#include "config.h"
#include "NanoSat.hpp"
#include "test/Test.hpp"
#include "ui/UI.hpp"
#include "utils/Settings.hpp"
#include "utils/OSSignalHandler.hpp"
#include "test/Calibration.hpp"

namespace test = sat::test;
using std::this_thread::sleep_for;

using namespace sat::utils;
using namespace sat::ui;

void run(sat::NanoSat& nanoSat, const Vector3f &target);
void runTests(const sat::NanoSat& nanoSat);
void calibrate(sat::NanoSat& nanoSat);
void runStandardMode(sat::NanoSat& satellite, const Vector3f &target);


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

      sleep_for(chrono::seconds(2));

		bool exit = false;
		while (!exit) {	// main program loop

         ui::clearConsole();	// clear screen

			// let the user choose operating mode
			const auto mode = ui::selectFromList({ "run standard mode",
                                                "run (select control axis)",
										                  "run tests",
										                  "calibrate",
			                                       "exit"});
			// run in the selected mode
			switch (mode) {
         case 2:  run(satellite, settings.global.target); break;
			case 3:	runTests(satellite); break;
			case 4:	calibrate(satellite); break;
         case 5:  break;
			default: runStandardMode(satellite, settings.global.target);
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


void runStandardMode(sat::NanoSat &satellite, const Vector3f &target) {

   std::cout << "Re-calibrating IMU... hold the nano sat in (0,0,0)... " << std::endl;
   satellite.getIMU()->reset();
   std::cout << "Done!" << std::endl;

   sat::trackMotionUntilKeyPress(satellite, 
      "Move the nano satellite to desired initial position. Press [ENTER] when in position.");

   sat::ctrl::Operation op;
   op.actuatorX = satellite.getDCMotor(Axis::X);
   op.actuatorY = satellite.getDCMotor(Axis::Y);
   op.actuatorZ = satellite.getDCMotor(Axis::Z);
   op.imu       = satellite.getIMU();
   op.finalState = target;

	if (satellite.move(op))
		std::cout << "Target reached!" << std::endl;
	else
		std::cout << "Couldn't reach target!" << std::endl;

	// do some operations here

}

void run(sat::NanoSat &satellite, const Vector3f &target) {

   std::cout << "Re-calibrating IMU... hold the nano sat in (0,0,0)... " << std::endl;
   satellite.getIMU()->reset();
   std::cout << "Done!" << std::endl;

   auto selected = ui::multipleSelectFromList({"Control X axis", "Control Y axis", "Control Z axis"});

   sat::trackMotionUntilKeyPress(satellite, 
      "Move the nano satellite to desired initial position. Press [ENTER] when in position.");

   sat::ctrl::Operation op;
   op.actuatorX = selected[0] ? satellite.getDCMotor(Axis::X) : nullptr;
   op.actuatorY = selected[1] ? satellite.getDCMotor(Axis::Y) : nullptr;
   op.actuatorZ = selected[2] ? satellite.getDCMotor(Axis::Z) : nullptr;
   op.imu       = satellite.getIMU();
   op.finalState = target;

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
		                        "gyroscope-imu comparison",
		                        "test imu tracking",
		                        "test power board",
		                        "back"});

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
      case 14: test.testGyroscopeImuError(); break;
      case 15: test.testIMUTracking();       break;
      case 16: test.powerBoard();            break;
      case 17: break;
		default: break;
		}

      exit = ui::yesNoAnswer("Exit test mode?");
	}

}

void calibrate(sat::NanoSat &nanoSat) {

   test::Calibration calib(&nanoSat);
   std::string header("-------- CALIBRATION MODE ---------\n");

   auto exit = false;
   while (!exit) {
      ui::clearConsole();
      cout << header << endl;

      const auto selection = ui::selectFromList({ "Calibrate gyroscope gains",
                                                  "Calibrate magnetometer offsets",
                                                  "Back"});
      switch (selection) {
      case 1: calib.calibrateGyroscopeGains();        break;
      case 2: calib.calibrateMagnetometerOffsets();   break;
      case 3: break;
      default: break;
      }

      exit = ui::yesNoAnswer("Exit calibration mode?");
   }
}