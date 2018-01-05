/*===========================================================================
										Nano satellite controller

// Copyright   : Copyright (c) 2017, Alberto Cenzato
					  All rights reserved.

// Based on: ADXL345 Driver for Arduino
				 https://github.com/jenschr/Arduino-libraries/tree/master/ADXL345

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU License.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU License V2 for more details: https://www.gnu.org/licenses/gpl-2.0.html

//============================================================================ */

#include "devices/Accelerometer.hpp"

#include <limits>

#include "utils/Settings.hpp"
#include "self_test/SelfTest.hpp"

#define TO_READ (6)	// num of bytes we are going to read each time (two bytes for each axis)

#pragma region registers
#define ADXL345_DEVID			 0x00
#define ADXL345_RESERVED1		 0x01
#define ADXL345_THRESH_TAP		 0x1d
#define ADXL345_OFSX			 0x1e
#define ADXL345_OFSY			 0x1f
#define ADXL345_OFSZ			 0x20
#define ADXL345_DUR				 0x21
#define ADXL345_LATENT			 0x22
#define ADXL345_WINDOW			 0x23
#define ADXL345_THRESH_ACT		 0x24
#define ADXL345_THRESH_INACT	 0x25
#define ADXL345_TIME_INACT		 0x26
#define ADXL345_ACT_INACT_CTL    0x27
#define ADXL345_THRESH_FF		 0x28
#define ADXL345_TIME_FF			 0x29
#define ADXL345_TAP_AXES		 0x2a
#define ADXL345_ACT_TAP_STATUS	 0x2b
#define ADXL345_BW_RATE			 0x2c
#define ADXL345_POWER_CTL		 0x2d
#define ADXL345_INT_ENABLE		 0x2e
#define ADXL345_INT_MAP			 0x2f
#define ADXL345_INT_SOURCE		 0x30
#define ADXL345_DATA_FORMAT		 0x31
#define ADXL345_DATAX0			 0x32
#define ADXL345_DATAX1			 0x33
#define ADXL345_DATAY0			 0x34
#define ADXL345_DATAY1			 0x35
#define ADXL345_DATAZ0			 0x36
#define ADXL345_DATAZ1			 0x37
#define ADXL345_FIFO_CTL		 0x38
#define ADXL345_FIFO_STATUS		 0x39
#define ADXL345_BW_1600			 0xF // 1111
#define ADXL345_BW_800			 0xE // 1110
#define ADXL345_BW_400			 0xD // 1101  
#define ADXL345_BW_200			 0xC // 1100
#define ADXL345_BW_100			 0xB // 1011  
#define ADXL345_BW_50			 0xA // 1010 
#define ADXL345_BW_25			 0x9 // 1001 
#define ADXL345_BW_12			 0x8 // 1000 
#define ADXL345_BW_6			 0x7 // 0111
#define ADXL345_BW_3			 0x6 // 0110
#pragma endregion registers

// Interrupt PINs - INT1: 0, INT2: 1
#define ADXL345_INT1_PIN 0x00
#define ADXL345_INT2_PIN 0x01

#pragma region interrupt_bits
#define ADXL345_INT_DATA_READY_BIT 0x07
#define ADXL345_INT_SINGLE_TAP_BIT 0x06
#define ADXL345_INT_DOUBLE_TAP_BIT 0x05
#define ADXL345_INT_ACTIVITY_BIT   0x04
#define ADXL345_INT_INACTIVITY_BIT 0x03
#define ADXL345_INT_FREE_FALL_BIT  0x02
#define ADXL345_INT_WATERMARK_BIT  0x01
#define ADXL345_INT_OVERRUNY_BIT   0x00
#pragma endregion interrupt_bits

#define ADXL345_OK			1 // no error
#define ADXL345_ERROR		0 // indicates error is 
#define ADXL345_NO_ERROR	0 // initial state
#define ADXL345_READ_ERROR	1 // problem reading accel
#define ADXL345_BAD_ARG		2 // bad method argument


using sat::utils::Vector3i;
using sat::utils::Vector3f;

namespace sat {
namespace device
{

const string Accelerometer::DEFAULT_DEV_NAME = "ADXL345_Accelerometer";

Accelerometer::Accelerometer() 
	: DeviceI2C(), status(true), error_code(ADXL345_NO_ERROR) {
	
   gains_ = { 0.00376390f, 0.00376009f, 0.00349265f };
}

Accelerometer::Accelerometer(const utils::AccelerometerSettigs& settings)
	: DeviceI2C(settings.deviceID, settings.address), status(true), error_code(ADXL345_NO_ERROR) {

	retrieveSettings(settings);

	setRangeSetting(2);
   Accelerometer::write8(ADXL345_POWER_CTL, 8); // power on
}


void Accelerometer::retrieveSettings(const utils::AccelerometerSettigs &settings) {
   offsets_ = settings.offsets;
   setGains(settings.gains);
}

vector<double> Accelerometer::read() const {
	auto val = readAccel();
	return {val[0], val[1], val[2]};
}

Vector3f Accelerometer::readAccel() const {
   auto values = readAccelRaw() - offsets_;
   for (auto i = 0; i < values.size; ++i)
      values[i] *= gains_[i];
	return values;
}

Vector3f Accelerometer::readAccelRaw() const {
	if (!available) {
		const auto nan = numeric_limits<float>::quiet_NaN();
		return {nan, nan, nan};
	}

	uint8_t buffer[6];
	readX(ADXL345_DATAX0, buffer, TO_READ); //read the acceleration data from the Accelerometer

	// each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
	// thus we are converting both bytes in to one 16-bit int
	return Vector3f { float(int16_t((buffer[1] << 8) | buffer[0])),
					      float(int16_t((buffer[3] << 8) | buffer[2])),
					      float(int16_t((buffer[5] << 8) | buffer[4])) };
}


	// Gets the range setting and return it into rangeSetting
// it can be 2, 4, 8 or 16
void Accelerometer::getRangeSetting(uint8_t* rangeSetting) const {
	uint8_t _b;
	readX(ADXL345_DATA_FORMAT, &_b, 1);
	*rangeSetting = _b & 0b00000011;
}

// Sets the range setting, possible values are: 2, 4, 8, 16
void Accelerometer::setRangeSetting(int val) {
	uint8_t _s;
	uint8_t _b;

	switch (val) {
	case 2:
		_s = 0b00000000;
		break;
	case 4:
		_s = 0b00000001;
		break;
	case 8:
		_s = 0b00000010;
		break;
	case 16:
		_s = 0b00000011;
		break;
	default:
		_s = 0b00000000;
	}
	readX(ADXL345_DATA_FORMAT, &_b, uint8_t(1));
	_s |= uint8_t(_b & uint8_t(0b11101100));
	write8(ADXL345_DATA_FORMAT, _s);
}
// gets the state of the SELF_TEST bit
bool Accelerometer::getSelfTestBit() const {
	return getRegisterBit(ADXL345_DATA_FORMAT, 7);
}

// Sets the SELF-TEST bit
// if set to 1 it applies a self-test force to the sensor causing a shift in the output data
// if set to 0 it disables the self-test force
void Accelerometer::setSelfTestBit(bool selfTestBit) {
	setRegisterBit(ADXL345_DATA_FORMAT, 7, selfTestBit);
}

// Gets the state of the SPI bit
bool Accelerometer::getSpiBit() const {
	return getRegisterBit(ADXL345_DATA_FORMAT, 6);
}

// Sets the SPI bit
// if set to 1 it sets the device to 3-wire mode
// if set to 0 it sets the device to 4-wire SPI mode
void Accelerometer::setSpiBit(bool spiBit) {
	setRegisterBit(ADXL345_DATA_FORMAT, 6, spiBit);
}

// Gets the state of the INT_INVERT bit
bool Accelerometer::getInterruptLevelBit() const {
	return getRegisterBit(ADXL345_DATA_FORMAT, 5);
}

// Sets the INT_INVERT bit
// if set to 0 sets the interrupts to active high
// if set to 1 sets the interrupts to active low
void Accelerometer::setInterruptLevelBit(bool interruptLevelBit) {
	setRegisterBit(ADXL345_DATA_FORMAT, 5, interruptLevelBit);
}

// Gets the state of the FULL_RES bit
bool Accelerometer::getFullResBit() const {
	return getRegisterBit(ADXL345_DATA_FORMAT, 3);
}

// Sets the FULL_RES bit
// if set to 1, the device is in full resolution mode, where the output resolution increases with the
//   g range set by the range bits to maintain a 4mg/LSB scal factor
// if set to 0, the device is in 10-bit mode, and the range buts determine the maximum g range
//   and scale factor
void Accelerometer::setFullResBit(bool fullResBit) {
	setRegisterBit(ADXL345_DATA_FORMAT, 3, fullResBit);
}

// Gets the state of the justify bit
bool Accelerometer::getJustifyBit() const {
	return getRegisterBit(ADXL345_DATA_FORMAT, 2);
}

// Sets the JUSTIFY bit
// if sets to 1 selects the left justified mode
// if sets to 0 selects right justified mode with sign extension
void Accelerometer::setJustifyBit(bool justifyBit) {
	setRegisterBit(ADXL345_DATA_FORMAT, 2, justifyBit);
}

// Sets the THRESH_TAP uint8_t value
// it should be between 0 and 255
// the scale factor is 62.5 mg/LSB
// A value of 0 may result in undesirable behavior
void Accelerometer::setTapThreshold(int tapThreshold) {
	tapThreshold = min(max(tapThreshold, 0), 255);
	write8(ADXL345_THRESH_TAP, uint8_t(tapThreshold));
}

// Gets the THRESH_TAP uint8_t value
// return value is comprised between 0 and 255
// the scale factor is 62.5 mg/LSB
int Accelerometer::getTapThreshold() const {
	uint8_t _b;
	readX(ADXL345_THRESH_TAP, &_b, 1);
	return int(_b);
}

Vector3f Accelerometer::getOffsets() const {
   return offsets_;
}

Vector3f Accelerometer::getGains() const {
   return gains_;
}

void Accelerometer::setOffsets(const Vector3f& offsets) {
   offsets_ = offsets;
}

// set/get the gain for each axis in Gs / count
void Accelerometer::setGains(const Vector3f& gains) {
	gains_ = gains;
}


// Sets the DUR uint8_t
// The DUR uint8_t contains an unsigned time value representing the maximum time
// that an event must be above THRESH_TAP threshold to qualify as a tap event
// The scale factor is 625µs/LSB
// A value of 0 disables the tap/float tap funcitons. Max value is 255.
void Accelerometer::setTapDuration(int tapDuration) {
	tapDuration = min(max(tapDuration, 0), 255);
	write8(ADXL345_DUR, uint8_t(tapDuration));
}

// Gets the DUR uint8_t
int Accelerometer::getTapDuration() const {
	uint8_t _b;
	readX(ADXL345_DUR, &_b, 1);
	return int(_b);
}

// Sets the latency (latent register) which contains an unsigned time value
// representing the wait time from the detection of a tap event to the start
// of the time window, during which a possible second tap can be detected.
// The scale factor is 1.25ms/LSB. A value of 0 disables the float tap function.
// It accepts a maximum value of 255.
void Accelerometer::setDoubleTapLatency(int floatTapLatency) {
	write8(ADXL345_LATENT, uint8_t(floatTapLatency));
}

// Gets the Latent value
int Accelerometer::getDoubleTapLatency() const {
	uint8_t _b;
	readX(ADXL345_LATENT, &_b, 1);
	return int(_b);
}

// Sets the Window register, which contains an unsigned time value representing
// the amount of time after the expiration of the latency time (Latent register)
// during which a second valud tap can begin. The scale factor is 1.25ms/LSB. A
// value of 0 disables the float tap function. The maximum value is 255.
void Accelerometer::setDoubleTapWindow(int floatTapWindow) {
	floatTapWindow = min(max(floatTapWindow, 0), 255);
	write8(ADXL345_WINDOW, uint8_t(floatTapWindow));
}

// Gets the Window register
int Accelerometer::getDoubleTapWindow() const {
	uint8_t _b;
	readX(ADXL345_WINDOW, &_b, 1);
	return int(_b);
}

// Sets the THRESH_ACT uint8_t which holds the threshold value for detecting activity.
// The data format is unsigned, so the magnitude of the activity event is compared
// with the value is compared with the value in the THRESH_ACT register. The scale
// factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the
// activity interrupt is enabled. The maximum value is 255.
void Accelerometer::setActivityThreshold(int activityThreshold) {
	activityThreshold = min(max(activityThreshold, 0), 255);
	write8(ADXL345_THRESH_ACT, uint8_t(activityThreshold));
}

// Gets the THRESH_ACT uint8_t
int Accelerometer::getActivityThreshold() const {
	uint8_t _b;
	readX(ADXL345_THRESH_ACT, &_b, 1);
	return int(_b);
}

// Sets the THRESH_INACT uint8_t which holds the threshold value for detecting inactivity.
// The data format is unsigned, so the magnitude of the inactivity event is compared
// with the value is compared with the value in the THRESH_INACT register. The scale
// factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the
// inactivity interrupt is enabled. The maximum value is 255.
void Accelerometer::setInactivityThreshold(int inactivityThreshold) {
	inactivityThreshold = min(max(inactivityThreshold, 0), 255);
	write8(ADXL345_THRESH_INACT, uint8_t(inactivityThreshold));
}

// Gets the THRESH_INACT uint8_t
int Accelerometer::getInactivityThreshold() const {
	uint8_t _b;
	readX(ADXL345_THRESH_INACT, &_b, 1);
	return int(_b);
}

// Sets the TIME_INACT register, which contains an unsigned time value representing the
// amount of time that acceleration must be less thant the value in the THRESH_INACT
// register for inactivity to be declared. The scale factor is 1sec/LSB. The value must
// be between 0 and 255.
void Accelerometer::setTimeInactivity(int timeInactivity) {
	timeInactivity = min(max(timeInactivity, 0), 255);
	write8(ADXL345_TIME_INACT, uint8_t(timeInactivity));
}

// Gets the TIME_INACT register
int Accelerometer::getTimeInactivity() const {
	uint8_t _b;
	readX(ADXL345_TIME_INACT, &_b, 1);
	return int(_b);
}

// Sets the THRESH_FF register which holds the threshold value, in an unsigned format, for
// free-fall detection. The root-sum-square (RSS) value of all axes is calculated and
// compared whith the value in THRESH_FF to determine if a free-fall event occured. The
// scale factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the free-fall
// interrupt is enabled. The maximum value is 255.
void Accelerometer::setFreeFallThreshold(int freeFallThreshold) {
	freeFallThreshold = min(max(freeFallThreshold, 0), 255);
	write8(ADXL345_THRESH_FF, uint8_t(freeFallThreshold));
}

// Gets the THRESH_FF register.
int Accelerometer::getFreeFallThreshold() const {
	uint8_t _b;
	readX(ADXL345_THRESH_FF, &_b, 1);
	return int(_b);
}

// Sets the TIME_FF register, which holds an unsigned time value representing the minimum
// time that the RSS value of all axes must be less than THRESH_FF to generate a free-fall
// interrupt. The scale factor is 5ms/LSB. A value of 0 may result in undesirable behavior if
// the free-fall interrupt is enabled. The maximum value is 255.
void Accelerometer::setFreeFallDuration(int freeFallDuration) {
	freeFallDuration = min(max(freeFallDuration, 0), 255);
	write8(ADXL345_TIME_FF, uint8_t(freeFallDuration));
}

// Gets the TIME_FF register.
int Accelerometer::getFreeFallDuration() const {
	uint8_t _b;
	readX(ADXL345_TIME_FF, &_b, 1);
	return int(_b);
}

bool Accelerometer::isActivityXEnabled() const {
	return getRegisterBit(ADXL345_ACT_INACT_CTL, 6);
}
bool Accelerometer::isActivityYEnabled() const {
	return getRegisterBit(ADXL345_ACT_INACT_CTL, 5);
}
bool Accelerometer::isActivityZEnabled() const {
	return getRegisterBit(ADXL345_ACT_INACT_CTL, 4);
}
bool Accelerometer::isInactivityXEnabled() const {
	return getRegisterBit(ADXL345_ACT_INACT_CTL, 2);
}
bool Accelerometer::isInactivityYEnabled() const {
	return getRegisterBit(ADXL345_ACT_INACT_CTL, 1);
}
bool Accelerometer::isInactivityZEnabled() const {
	return getRegisterBit(ADXL345_ACT_INACT_CTL, 0);
}

void Accelerometer::setActivityX(bool state) {
	setRegisterBit(ADXL345_ACT_INACT_CTL, 6, state);
}
void Accelerometer::setActivityY(bool state) {
	setRegisterBit(ADXL345_ACT_INACT_CTL, 5, state);
}
void Accelerometer::setActivityZ(bool state) {
	setRegisterBit(ADXL345_ACT_INACT_CTL, 4, state);
}
void Accelerometer::setInactivityX(bool state) {
	setRegisterBit(ADXL345_ACT_INACT_CTL, 2, state);
}
void Accelerometer::setInactivityY(bool state) {
	setRegisterBit(ADXL345_ACT_INACT_CTL, 1, state);
}
void Accelerometer::setInactivityZ(bool state) {
	setRegisterBit(ADXL345_ACT_INACT_CTL, 0, state);
}

bool Accelerometer::isActivityAc() const {
	return getRegisterBit(ADXL345_ACT_INACT_CTL, 7);
}
bool Accelerometer::isInactivityAc() const {
	return getRegisterBit(ADXL345_ACT_INACT_CTL, 3);
}

void Accelerometer::setActivityAc(bool state) {
	setRegisterBit(ADXL345_ACT_INACT_CTL, 7, state);
}
void Accelerometer::setInactivityAc(bool state) {
	setRegisterBit(ADXL345_ACT_INACT_CTL, 3, state);
}

bool Accelerometer::getSuppressBit() const {
	return getRegisterBit(ADXL345_TAP_AXES, 3);
}
void Accelerometer::setSuppressBit(bool state) {
	setRegisterBit(ADXL345_TAP_AXES, 3, state);
}

bool Accelerometer::isTapDetectionOnX() const {
	return getRegisterBit(ADXL345_TAP_AXES, 2);
}
void Accelerometer::setTapDetectionOnX(bool state) {
	setRegisterBit(ADXL345_TAP_AXES, 2, state);
}
bool Accelerometer::isTapDetectionOnY() const {
	return getRegisterBit(ADXL345_TAP_AXES, 1);
}
void Accelerometer::setTapDetectionOnY(bool state) {
	setRegisterBit(ADXL345_TAP_AXES, 1, state);
}
bool Accelerometer::isTapDetectionOnZ() const {
	return getRegisterBit(ADXL345_TAP_AXES, 0);
}
void Accelerometer::setTapDetectionOnZ(bool state) {
	setRegisterBit(ADXL345_TAP_AXES, 0, state);
}

bool Accelerometer::isActivitySourceOnX() const {
	return getRegisterBit(ADXL345_ACT_TAP_STATUS, 6);
}
bool Accelerometer::isActivitySourceOnY() const {
	return getRegisterBit(ADXL345_ACT_TAP_STATUS, 5);
}
bool Accelerometer::isActivitySourceOnZ() const {
	return getRegisterBit(ADXL345_ACT_TAP_STATUS, 4);
}

bool Accelerometer::isTapSourceOnX() const {
	return getRegisterBit(ADXL345_ACT_TAP_STATUS, 2);
}
bool Accelerometer::isTapSourceOnY() const {
	return getRegisterBit(ADXL345_ACT_TAP_STATUS, 1);
}
bool Accelerometer::isTapSourceOnZ() const {
	return getRegisterBit(ADXL345_ACT_TAP_STATUS, 0);
}

bool Accelerometer::isAsleep() const {
	return getRegisterBit(ADXL345_ACT_TAP_STATUS, 3);
}

bool Accelerometer::isLowPower() const {
	return getRegisterBit(ADXL345_BW_RATE, 4);
}
void Accelerometer::setLowPower(bool state) {
	setRegisterBit(ADXL345_BW_RATE, 4, state);
}

float Accelerometer::getRate() const {
	uint8_t _b;
	readX(ADXL345_BW_RATE, &_b, 1);
	_b &= 0b00001111;
	return pow(2.f, int(_b) - 6) * 6.25f;
}

void Accelerometer::setRate(float rate) {
	uint8_t _b;
	int v = int(rate / 6.25);
	int r = 0;
	while (v >>= 1) {
		r++;
	}
	if (r <= 9) {
		readX(ADXL345_BW_RATE, &_b, uint8_t(1));
		const uint8_t _s = (r + 6) | _b & uint8_t(0b11110000);
		write8(ADXL345_BW_RATE, _s);
	}
}

void Accelerometer::set_bw(uint8_t bw_code) {
	if ((bw_code < ADXL345_BW_3) || (bw_code > ADXL345_BW_1600)) {
		status = false;
		error_code = ADXL345_BAD_ARG;
	}
	else {
		write8(ADXL345_BW_RATE, bw_code);
	}
}

uint8_t Accelerometer::get_bw_code() const {
	uint8_t bw_code;
	readX(ADXL345_BW_RATE, &bw_code, 1);
	return bw_code;
}

uint8_t Accelerometer::getInterruptSource() const {
	uint8_t _b;
	readX(ADXL345_INT_SOURCE, &_b, 1);
	return _b;
}

bool Accelerometer::getInterruptSource(uint8_t interruptBit) const {
	return getRegisterBit(ADXL345_INT_SOURCE, interruptBit);
}

bool Accelerometer::getInterruptMapping(uint8_t interruptBit) const {
	return getRegisterBit(ADXL345_INT_MAP, interruptBit);
}

// Set the mapping of an interrupt to pin1 or pin2
// eg: setInterruptMapping(ADXL345_INT_DOUBLE_TAP_BIT,ADXL345_INT2_PIN);
void Accelerometer::setInterruptMapping(uint8_t interruptBit, bool interruptPin) {
	setRegisterBit(ADXL345_INT_MAP, interruptBit, interruptPin);
}

bool Accelerometer::isInterruptEnabled(uint8_t interruptBit) const {
	return getRegisterBit(ADXL345_INT_ENABLE, interruptBit);
}

void Accelerometer::setInterrupt(uint8_t interruptBit, bool state) {
	setRegisterBit(ADXL345_INT_ENABLE, interruptBit, state);
}

void Accelerometer::setRegisterBit(uint8_t regAdress, int bitPos, bool state) {
	uint8_t _b;
	readX(regAdress, &_b, 1);
	if (state) {
		_b |= uint8_t(1 << bitPos);  // forces nth bit of _b to be 1.  all other bits left alone.
	}
	else {
		_b &= uint8_t(~(1 << bitPos)); // forces nth bit of _b to be 0.  all other bits left alone.
	}
	write8(regAdress, _b);
}

bool Accelerometer::getRegisterBit(uint8_t regAdress, int bitPos) const {
	uint8_t _b;
	read8(regAdress, _b);
	return ((_b >> bitPos) & 1);
}

std::unique_ptr<Accelerometer> Accelerometer::create(const utils::AccelerometerSettigs & settings) {
	return std::make_unique<Accelerometer>(settings);
}


// ---------- Inherited functions ----------
#pragma region inherited_functions

TestResult Accelerometer::testConnection() {
	auto test = ConnectionSelfTest::create(deviceID);
	uint8_t value;
	if (!read8(ADXL345_DEVID, value)) {
		test->errorLevel = ErrorLevel::warning;
		test->additionalInfo += " | Read error!";
		return TestResult(test);
	}
	if (value != 0b11100101)
		test->errorLevel = ErrorLevel::warning;
	
	return TestResult(test);
}

#pragma endregion inherited_functions

} // namespace device
} // namespace sat