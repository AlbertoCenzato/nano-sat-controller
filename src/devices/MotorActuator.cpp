#include "devices/MotorActuator.hpp"

#include <iostream>
#include <cmath>
#include <thread>

#include "utils/Settings.hpp"
#include "self_test/SelfTest.hpp"
#include "utils/Exceptions.hpp"


#define MOTOR_SINGLE		 1
#define MOTOR_DOUBLE		 2
#define MOTOR_INTERLEAVE 3
#define MOTOR_MICROSTEP  4

// Registers / etc.
#pragma region registers
#define MOTOR_MODE1			 0x00
#define MOTOR_MODE2			 0x01
#define MOTOR_SUBADR1		 0x02
#define MOTOR_SUBADR2		 0x03
#define MOTOR_SUBADR3		 0x04
#define MOTOR_PRESCALE		 0xFE
#define MOTOR_LED0_ON_L		 0x06
#define MOTOR_LED0_ON_H		 0x07
#define MOTOR_LED0_OFF_L	 0x08
#define MOTOR_LED0_OFF_H	 0x09
#define MOTOR_ALL_LED_ON_L  0xFA
#define MOTOR_ALL_LED_ON_H	 0xFB
#define MOTOR_ALL_LED_OFF_L 0xFC
#define MOTOR_ALL_LED_OFF_H 0xFD
#pragma endregion registers

// Bits 
#define MOTOR_RESTART 0x80
#define MOTOR_SLEEP   0x10
#define MOTOR_ALLCALL 0x01
#define MOTOR_INVRT   0x10
#define MOTOR_OUTDRV  0x04

#define MOTOR_CONNECTION_REGISTER_VALUE 0x0404

using this_thread::sleep_for;

namespace sat {
namespace device {

const string MotorActuator::DEFAULT_DEV_NAME = "Motor_actuator";

// ---------- Constructors ----------
#pragma region constructors

MotorActuator::MotorActuator() : DeviceI2C() { }

MotorActuator::MotorActuator(const utils::MotorSettings &settings)
	: DeviceI2C(settings.deviceID, settings.address) {

	//----- PWM constructor -----

	//self.i2c = get_i2c_device(address, i2c, i2c_bus);
	//logger.debug("Reseting PCA9685 MODE1 (without SLEEP) and MODE2");

	setAllPWM(0, 0);
	write8(MOTOR_MODE2, MOTOR_OUTDRV);
	write8(MOTOR_MODE1, MOTOR_ALLCALL);

	sleep_for(chrono::milliseconds(5));      // wait for oscillator

	uint8_t mode1;
	read8(MOTOR_MODE1, mode1);
	mode1 = mode1 & static_cast<uint8_t>(~MOTOR_SLEEP); // wake up(reset sleep)
	write8(MOTOR_MODE1, mode1);

	sleep_for(chrono::milliseconds(5));      // wait for oscillator

														  // ----- MOTOR constructor -----

	motornum = settings.num;

	if (settings.num == 0) {
		PWMpin = 8;
		IN1pin = 9;
		IN2pin = 10;
	}
	else if (settings.num == 1) {
		PWMpin = 13;
		IN1pin = 12;
		IN2pin = 11;
	}
	else if (settings.num == 2) {
		PWMpin = 2;
		IN1pin = 3;
		IN2pin = 4;
	}
	else if (settings.num == 3) {
		PWMpin = 7;
		IN1pin = 6;
		IN2pin = 5;
	}
	else {
		cerr << "MotorHAT Motor must be between 1 and 4 inclusive" << endl;
		PWMpin = 0;
		IN1pin = 0;
		IN2pin = 0;
	}
}

MotorActuator::~MotorActuator() {
	run(MOTOR_RELEASE);
}

#pragma endregion constructors

//---------- Public member functions ----------
#pragma region functions

void MotorActuator::run(int command) {

	switch(command) {
	case MOTOR_FORWARD:
		setPWM(IN2pin, 0, 4096);
		setPWM(IN1pin, 4096, 0);
		running = true;
		break;
	case MOTOR_BACKWARD:
		setPWM(IN1pin, 0, 4096);
		setPWM(IN2pin, 4096, 0);
		running = true;
		break;
	case MOTOR_RELEASE:
		setPWM(IN1pin, 0, 4096);
		setPWM(IN2pin, 0, 4096);
		running = false;
	}
}

void MotorActuator::setSpeed(int speed) {
	if (speed < 0)
		speed = 0;
	if (speed > 255)
		speed = 255;
	setPWM(PWMpin, 0, speed * 16);
	this->speed = speed * 16;
}

void MotorActuator::act(float action) {
	if (action < 0)
		run(MOTOR_FORWARD);
	else if (action == 0)
		run(MOTOR_RELEASE);
	else
		run(MOTOR_BACKWARD);

	setSpeed(static_cast<int>(abs(action)));
}

std::unique_ptr<MotorActuator> MotorActuator::create(const utils::MotorSettings & settings) {
	return std::make_unique<MotorActuator>(settings);
}


void MotorActuator::softwareReset(/*cls, i2c = None, i2c_bus = None*/) {

	throw utils::NotImplementedException();

	//general_call_i2c = get_i2c_device(0x00, i2c, i2c_bus);
	//general_call_i2c.writeRaw8(0x06);     // SWRST
}
#pragma endregion functions

// ---------- Private member functions ----------
#pragma region priv_functions

void MotorActuator::setPWMFreq(int freq) {
	const float prescaleval = (25000000.0 / 4096.0f) / float(freq) - 1.0f;  // 25MHz
	const auto prescale = floor(prescaleval + 0.5f);
	uint8_t oldmode;
	read8(MOTOR_MODE1, oldmode);
	auto newmode = uint8_t((oldmode & 0x7F) | 0x10);     // sleep
	write8(MOTOR_MODE1, newmode);     // go to sleep
	write8(MOTOR_PRESCALE, uint8_t(floor(prescale)));
	write8(MOTOR_MODE1, oldmode);

	sleep_for(chrono::milliseconds(5));

	write8(MOTOR_MODE1, oldmode | uint8_t(0x80));
}

void MotorActuator::setPWM(int channel, int on, int off) {
	write8(uint8_t(MOTOR_LED0_ON_L + 4 * channel), uint8_t(on & 0xFF));
	write8(uint8_t(MOTOR_LED0_ON_H + 4 * channel), uint8_t(on >> 8));
	write8(uint8_t(MOTOR_LED0_OFF_L + 4 * channel), uint8_t(off & 0xFF));
	write8(uint8_t(MOTOR_LED0_OFF_H + 4 * channel), uint8_t(off >> 8));
}

void MotorActuator::setAllPWM(int on, int off) {
	write8(MOTOR_ALL_LED_ON_L,  uint8_t(on & 0xFF));
	write8(MOTOR_ALL_LED_ON_H,  uint8_t(on >> 8));
	write8(MOTOR_ALL_LED_OFF_L, uint8_t(off & 0xFF));
	write8(MOTOR_ALL_LED_OFF_H, uint8_t(off >> 8));
}

TestResult MotorActuator::testConnection() {
	auto test = ConnectionSelfTest::create(deviceID);

	uint16_t val;
	if (!read16(MOTOR_ALLCALL, val)) {
		test->errorLevel = ErrorLevel::warning;
		test->additionalInfo += " | Read error!";
		return TestResult(test);
	}

	if (val != MOTOR_CONNECTION_REGISTER_VALUE)
		test->errorLevel = ErrorLevel::warning;

	return TestResult(test);
}

#pragma endregion priv_functions
} // end device
} // end sat