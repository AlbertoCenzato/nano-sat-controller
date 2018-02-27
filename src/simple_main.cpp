#include <iostream>

#include "NanoSat.hpp"
#include "utils/Settings.hpp"

using namespace sat;
using utils::Settings;
using ctrl::Operation;

int __main__()
{
	const Settings settings;	// load settings from configuration file

	NanoSat satellite(settings.nanoSat); // setup nano satellite and all attached devices

	auto motor = satellite.getDCMotor(Axis::X);

	// prepare a maneuver
   Operation op;   // define an operation to do
   op.actuatorX = satellite.getCoil(Axis::X);  // retrieve x-axis coil 
   op.actuatorY = satellite.getCoil(Axis::Y);  // retrieve y-axis coil
   op.actuatorZ = satellite.getCoil(Axis::Z);  // retrieve z-axis coil
   op.imu = satellite.getIMU();   // retrieve imu
   op.targetState = { 0.f, 0.f, 0.f };
    
	auto success = satellite.move(op);	// execute maneuver
	if (success) {
		std::cout << "Operation executed!" << std::endl;
	}
	else {
		std::cout << "Execution failed!" << std::endl;
	}

   auto imu = satellite.getIMU();
	auto gyro = imu->getGyroscope(); // get specific imu sensor

	auto readings = gyro->readGyro(); // read values from gyroscope

	// print values
	std::cout << "Gyroscope values:" << std::endl;
	for (auto val : readings) {
		std::cout << val << std::endl;
	}
	
	return 0;
}
