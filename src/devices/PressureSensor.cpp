#include "devices/PressureSensor.hpp"

#include <thread>

#include "utils/Settings.hpp"
#include "self_test/SelfTest.hpp"

#pragma region registers
#define AC1_REGISTER 0xAA
#define AC2_REGISTER 0xAC
#define AC3_REGISTER 0xAE
#define AC4_REGISTER 0xB0
#define AC5_REGISTER 0xB2
#define AC6_REGISTER 0xB4

#define B1_REGISTER 0xB6
#define B2_REGISTER 0xB8

#define MB_REGISTER 0xBA
#define MC_REGISTER 0xBC
#define MD_REGISTER 0xBE
#pragma endregion registers



using this_thread::sleep_for;

namespace sat {
namespace device {

const string PressureSensor::DEFAULT_DEV_NAME = "BMP085_pressure_sensor";

// ---------- Constructors ----------
#pragma region constructors
PressureSensor::PressureSensor() : DeviceI2C() { }

PressureSensor::PressureSensor(gnublin_i2c *bus, const utils::PressureSensorSettings& settings)
	: DeviceI2C(settings.deviceID, bus, settings.address) {
	
	read16(AC1_REGISTER, ac1);
	read16(AC2_REGISTER, ac2);
	read16(AC3_REGISTER, ac3);
	read16(AC4_REGISTER, ac4);
	read16(AC5_REGISTER, ac5);
	read16(AC6_REGISTER, ac6);
	read16(B1_REGISTER,  b1);
	read16(B2_REGISTER,  b2);
	read16(MB_REGISTER,  mb);
	read16(MC_REGISTER,  mc);
	read16(MD_REGISTER,  md);
}

PressureSensor::~PressureSensor() { }

#pragma endregion constructors


// ---------- Public member functions ----------
#pragma region functions

vector<double> PressureSensor::read() const {
	return { double(readPressure()) };
}

long PressureSensor::readPressure() const {
	return readPressureRaw();
}

long PressureSensor::readPressureRaw() const {
	if (!available)
		return std::numeric_limits<long>::quiet_NaN();

	return get_pressure(readUP());
}

std::unique_ptr<PressureSensor> PressureSensor::create(gnublin_i2c * bus, const utils::PressureSensorSettings & settings) {
	return std::make_unique<PressureSensor>(bus, settings);
}

#pragma endregion functions


// ---------- Private member functions ----------

#pragma region priv_functions


long PressureSensor::get_temperature(unsigned int ut) const {
	long x1 = ((long(ut) - long(ac6)) * long(ac5)) >> 15;
	long x2 = (long(mc) << 11) / (x1 + md);

	return x1 + x2;
}


long PressureSensor::get_pressure(unsigned long up) const {
	auto _b5 = get_temperature(readUT());
	long p;

	auto b6 = _b5 - 4000;
	// Calculate B3
	long x1 = (b2 * (b6 * b6) >> 12) >> 11;
	long x2 = (ac2 * b6) >> 11;
	long x3 = x1 + x2;
	long b3 = (((long(ac1) * 4 + x3) << OSS) + 2) >> 2;

	// Calculate B4
	x1 = (ac3 * b6) >> 13;
	x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	unsigned long b4 = (ac4 * static_cast<unsigned long>(x3 + 32768)) >> 15;

	unsigned long b7 = (up - b3) * (50000 >> OSS);
	if (b7 < 0x80000000)
		p = (b7 << 1) / b4;
	else
		p = (b7 / b4) << 1;

	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	p += (x1 + x2 + 3791) >> 4;

	return p;
}


unsigned int PressureSensor::readUT() const {
	uint16_t ut;

	// Write 0x2E into Register 0xF4
	// This requests a temperature reading
	write8_const(0xF4, 0x2E);
	
	sleep_for(chrono::milliseconds(5)); // Wait at least 4.5ms

	// Read two bytes from registers 0xF6 and 0xF7
	read16(0xF6, ut);
	return ut;
}


unsigned long PressureSensor::readUP() const {
	unsigned char msb, lsb, xlsb;

	// Write 0x34+(OSS<<6) into register 0xF4
	// Request a pressure reading w/ oversampling setting
	write8_const(0xF4, 0x34 + (OSS << 6));

	// Wait for conversion, delay time dependent on OSS
	sleep_for(chrono::milliseconds(2 + (3 << OSS)));

	// Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
	read8(0xF6, msb);
	read8(0xF7, lsb);
	read8(0xF8, xlsb);

	unsigned long up = ((static_cast<unsigned long>(msb) << 16) |
						(static_cast<unsigned long>(lsb) << 8) |
						 static_cast<unsigned long>(xlsb)) >> (8 - OSS);

	return up;
}


// WARNING: not tested when a real device is connected.
TestResult PressureSensor::testConnection() {
	auto test = ConnectionSelfTest::create(deviceID);
	
	uint8_t val = 0;
	if (!read8(0xF4, val)) {
		test->errorLevel = ErrorLevel::warning;
		test->additionalInfo = "Read error!";
		return TestResult(test);
	}
	switch(OSS) {
	case 0: if (val != 0x34)
				test->errorLevel = ErrorLevel::warning;
			break;
	case 1: if (val != 0x74)
				test->errorLevel = ErrorLevel::warning;
			break;
	case 2: if (val != 0xB4)
				test->errorLevel = ErrorLevel::warning;
			break;
	case 3:	if (val != 0xF4)
				test->errorLevel = ErrorLevel::warning;
			break;
	default: test->errorLevel = ErrorLevel::warning;
			 test->additionalInfo = "OSS not in [0,3] interval!";
	}
	
	return TestResult(test);
}


#pragma endregion priv_functions

	} // end device
} // end sat
