/*===========================================================================
Nano satellite controller

// Copyright: Copyright (c) 2017, Alberto Cenzato
All rights reserved.

// Licence: GNU

// Based on: ITG-3200/I2C library v0.5 for Arduino by Filipe Vieira
http://code.google.com/p/itg-3200driver

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU License.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU License V3 for more details: https://www.gnu.org/licenses/gpl-3.0.html

//============================================================================ */

#include "devices/Gyroscope.hpp"

#include <thread>

#include "utils/Settings.hpp"
#include "self_test/SelfTest.hpp"


#define GYROSTART_UP_DELAY  70    // 50ms from gyro startup + 20ms register r/w startup

#pragma region registers
#define WHO_AM_I           0x00  // RW   SETUP: I2C address   
#define SMPLRT_DIV         0x15  // RW   SETUP: Sample Rate Divider       
#define DLPF_FS            0x16  // RW   SETUP: Digital Low Pass Filter/ Full Scale range
#define INT_CFG            0x17  // RW   Interrupt: Configuration
#define INT_STATUS         0x1A  // R	Interrupt: Status
#define TEMP_OUT           0x1B  // R	SENSOR: Temperature 2bytes
#define GYRO_XOUT          0x1D  // R	SENSOR: Gyro X 2bytes  
#define GYRO_YOUT          0x1F  // R	SENSOR: Gyro Y 2bytes
#define GYRO_ZOUT          0x21  // R	SENSOR: Gyro Z 2bytes
#define PWR_MGM            0x3E  // RW	Power Management
#pragma endregion registers

#pragma region bitmaps
#define DLPFFS_FS_SEL             0x18  // 00011000
#define DLPFFS_DLPF_CFG           0x07  // 00000111
#define INTCFG_ACTL               0x80  // 10000000
#define INTCFG_OPEN               0x40  // 01000000
#define INTCFG_LATCH_INT_EN       0x20  // 00100000
#define INTCFG_INT_ANYRD_2CLEAR   0x10  // 00010000
#define INTCFG_ITG_RDY_EN         0x04  // 00000100
#define INTCFG_RAW_RDY_EN         0x01  // 00000001
#define INTSTATUS_ITG_RDY         0x04  // 00000100
#define INTSTATUS_RAW_DATA_RDY    0x01  // 00000001
#define PWRMGM_HRESET             0x80  // 10000000
#define PWRMGM_SLEEP              0x40  // 01000000
#define PWRMGM_STBY_XG            0x20  // 00100000
#define PWRMGM_STBY_YG            0x10  // 00010000
#define PWRMGM_STBY_ZG            0x08  // 00001000
#define PWRMGM_CLK_SEL            0x07  // 00000111
#pragma endregion bitmaps

#pragma region register_params
// Sample Rate Divider
#define NOSRDIVIDER         0 // default    FsampleHz=SampleRateHz/(divider+1)
// Gyro Full Scale Range
#define RANGE2000           3   // default
// Digital Low Pass Filter BandWidth and SampleRate
#define BW256_SR8           0   // default    256Khz BW and 8Khz SR
#define BW188_SR1           1
#define BW098_SR1           2
#define BW042_SR1           3
#define BW020_SR1           4
#define BW010_SR1           5
#define BW005_SR1           6
// Interrupt Active logic lvl
#define ACTIVE_ONHIGH       0 // default
#define ACTIVE_ONLOW        1
// Interrupt drive type
#define PUSH_PULL           0 // default
#define OPEN_DRAIN          1
// Interrupt Latch mode
#define PULSE_50US          0 // default
#define UNTIL_INT_CLEARED   1
// Interrupt Latch clear method
#define READ_STATUSREG      0 // default
#define READ_ANYREG         1
// Power management
#define NORMAL              0 // default
#define STANDBY             1
// Clock Source - user parameters
#define INTERNALOSC         0   // default
#define PLL_XGYRO_REF       1
#define PLL_YGYRO_REF       2
#define PLL_ZGYRO_REF       3
#define PLL_EXTERNAL32      4   // 32.768 kHz
#define PLL_EXTERNAL19      5   // 19.2 Mhz
#pragma endregion register_params


using this_thread::sleep_for;
using sat::utils::Vector3i;
using sat::utils::Vector3f;

namespace sat {
namespace device
{

const string Gyroscope::DEFAULT_DEV_NAME = "ITG3200_Gyroscope_sensor";

// ---------- Constructors ----------
#pragma region constructors

Gyroscope::Gyroscope() : DeviceI2C() { }

Gyroscope::Gyroscope(const utils::GyroscopeSettings& settings)
	: DeviceI2C(settings.deviceID, settings.address) {
	
   setGains(settings.gains);
   setOffsets(settings.offsets);

	// init
	setSampleRateDiv(NOSRDIVIDER);
	setFSRange(RANGE2000);
	setFilterBW(BW256_SR8);
	setClockSource(PLL_XGYRO_REF);
	setITGReady(true);
	setRawDataReady(true);

	setDevAddr(address);
	sleep_for(chrono::milliseconds(GYROSTART_UP_DELAY)); // startup 
}

#pragma endregion constructors


// ---------- Member functions ----------
#pragma region functions

Vector3f Gyroscope::getOffsets() const { return offsets_; }
Vector3f Gyroscope::getGains()   const { return gains_;   }

void Gyroscope::setGains(const Vector3f& gains)     { gains_   = gains;   }
void Gyroscope::setOffsets(const Vector3f& offsets) { offsets_ = offsets; }


Vector3f Gyroscope::readGyro() const {
   auto values = readGyroRaw() - offsets_;
   for (auto i = 0; i < values.size; ++i)
      values[i] *= gains_[i] / 14.375;
   return values;
}

Vector3f Gyroscope::readGyroRaw() const {
   if (!available) {
      const auto nan = numeric_limits<float>::quiet_NaN();
      return { nan, nan, nan };
   }
   uint8_t buffer[6];
   readX(GYRO_XOUT, buffer, 6);

   return Vector3f{ float(int16_t((buffer[0] << 8) | buffer[1])),
                    float(int16_t((buffer[2] << 8) | buffer[3])),
                    float(int16_t((buffer[4] << 8) | buffer[5])) };
}

vector<double> Gyroscope::read() const {
   auto val = readGyro();
   return { val[0], val[1], val[2] };
}


void Gyroscope::setDevAddr(unsigned int  _addr) {
	write8(WHO_AM_I, _addr);
	this->address = _addr;
}

uint8_t Gyroscope::getSampleRateDiv() const {
	uint8_t val;
	read8(SMPLRT_DIV, val);
	return val;
}

void Gyroscope::setSampleRateDiv(uint8_t _SampleRate) {
	write8(SMPLRT_DIV, _SampleRate);
}

uint8_t Gyroscope::getFSRange() const {
	uint8_t val;
	read8(DLPF_FS, val);
	return ((val & DLPFFS_FS_SEL) >> 3);
}

void Gyroscope::setFSRange(uint8_t _Range) {
	uint8_t val;
	read8(DLPF_FS, val);
	write8(DLPF_FS, ((val & ~DLPFFS_FS_SEL) | (_Range << 3)));
}

uint8_t Gyroscope::getFilterBW() const {
	uint8_t val;
	read8(DLPF_FS, val);
	return (val & DLPFFS_DLPF_CFG);
}

void Gyroscope::setFilterBW(uint8_t _BW) {
	uint8_t val;
	read8(DLPF_FS, val);
	write8(DLPF_FS, ((val & ~DLPFFS_DLPF_CFG) | _BW));
}

bool Gyroscope::isINTActiveOnLow() const {
	uint8_t val;
	read8(INT_CFG, val);
	return ((val & INTCFG_ACTL) >> 7);
}

void Gyroscope::setINTLogiclvl(bool _State) {
	uint8_t val;
	read8(INT_CFG, val);
	write8(INT_CFG, ((val & ~INTCFG_ACTL) | (_State << 7)));
}

bool Gyroscope::isINTOpenDrain() const {
	uint8_t val;
	read8(INT_CFG, val);
	return ((val & INTCFG_OPEN) >> 6);
}

void Gyroscope::setINTDriveType(bool _State) {
	uint8_t val;
	read8(INT_CFG, val);
	write8(INT_CFG, ((val & ~INTCFG_OPEN) | _State << 6));
}

bool Gyroscope::isLatchUntilCleared() const {
	uint8_t val;
	read8(INT_CFG, val);
	return ((val & INTCFG_LATCH_INT_EN) >> 5);
}

void Gyroscope::setLatchMode(bool _State) {
	uint8_t val;
	read8(INT_CFG, val);
	write8(INT_CFG, ((val & ~INTCFG_LATCH_INT_EN) | _State << 5));
}

bool Gyroscope::isAnyRegClrMode() const {
	uint8_t val;
	read8(INT_CFG, val);
	return ((val & INTCFG_INT_ANYRD_2CLEAR) >> 4);
}

void Gyroscope::setLatchClearMode(bool _State) {
	uint8_t val;
	read8(INT_CFG, val);
	write8(INT_CFG, ((val & ~INTCFG_INT_ANYRD_2CLEAR) | _State << 4));
}

bool Gyroscope::isITGReadyOn() const {
	uint8_t val;
	read8(INT_CFG, val);
	return ((val & INTCFG_ITG_RDY_EN) >> 2);
}

void Gyroscope::setITGReady(bool _State) {
	uint8_t val;
	read8(INT_CFG, val);
	write8(INT_CFG, ((val & ~INTCFG_ITG_RDY_EN) | _State << 2));
}

bool Gyroscope::isRawDataReadyOn() const {
	uint8_t val;
	read8(INT_CFG, val);
	return (val & INTCFG_RAW_RDY_EN);
}

void Gyroscope::setRawDataReady(bool _State) {
	uint8_t val;
	read8(INT_CFG, val);
	write8(INT_CFG, ((val & ~INTCFG_RAW_RDY_EN) | _State));
}

bool Gyroscope::isITGReady() const {
	uint8_t val;
	read8(INT_STATUS, val);
	return ((val & INTSTATUS_ITG_RDY) >> 2);
}

bool Gyroscope::isRawDataReady() const {
	uint8_t val;
	read8(INT_STATUS, val);
	return (val & INTSTATUS_RAW_DATA_RDY);
}

void Gyroscope::readTemp(float *_Temp) const {
	uint8_t val[2];
	readX(TEMP_OUT, val, 2);
	*_Temp = 35 + (((val[0] << 8) | val[1]) + 13200) / 280.0;    // F=C*9/5+32  
}


void Gyroscope::reset() {
	write8(PWR_MGM, PWRMGM_HRESET);
	sleep_for(chrono::milliseconds(GYROSTART_UP_DELAY)); //gyro startup 
}

bool Gyroscope::isLowPower() const {
	uint8_t val;
	read8(PWR_MGM, val);
	return (val & PWRMGM_SLEEP) >> 6;
}

void Gyroscope::setPowerMode(bool _State) {
	uint8_t val;
	read8(PWR_MGM, val);
	write8(PWR_MGM, ((val & ~PWRMGM_SLEEP) | _State << 6));
}

bool Gyroscope::isXgyroStandby() const {
	uint8_t val;
	read8(PWR_MGM, val);
	return (val & PWRMGM_STBY_XG) >> 5;
}

bool Gyroscope::isYgyroStandby() const {
	uint8_t val;
	read8(PWR_MGM, val);
	return (val & PWRMGM_STBY_YG) >> 4;
}

bool Gyroscope::isZgyroStandby() const {
	uint8_t val;
	read8(PWR_MGM, val);
	return (val & PWRMGM_STBY_ZG) >> 3;
}

void Gyroscope::setXgyroStandby(bool _Status) {
	uint8_t val;
	read8(PWR_MGM, val);
	write8(PWR_MGM, ((val & PWRMGM_STBY_XG) | _Status << 5));
}

void Gyroscope::setYgyroStandby(bool _Status) {
	uint8_t val;
	read8(PWR_MGM, val);
	write8(PWR_MGM, ((val & PWRMGM_STBY_YG) | _Status << 4));
}

void Gyroscope::setZgyroStandby(bool _Status) {
	uint8_t val;
	read8(PWR_MGM, val);
	write8(PWR_MGM, uint8_t((val & PWRMGM_STBY_ZG) | _Status << 3));
}

uint8_t Gyroscope::getClockSource() const {
	uint8_t val;
	read8(PWR_MGM, val);
	return (val & PWRMGM_CLK_SEL);
}

void Gyroscope::setClockSource(uint8_t _CLKsource) {
	uint8_t val;
	read8(PWR_MGM, val);
	write8(PWR_MGM, uint8_t((val & ~PWRMGM_CLK_SEL) | _CLKsource));
}
std::unique_ptr<Gyroscope> Gyroscope::create(const utils::GyroscopeSettings & settings) {
	return std::make_unique<Gyroscope>(settings);
}
#pragma endregion functions


// ---------- Inherited functions ----------
#pragma region inherited_functions

TestResult Gyroscope::testConnection() {
	auto test = ConnectionSelfTest::create(deviceID);
	uint8_t value;
	if (!read8(WHO_AM_I, value)) {
		test->errorLevel = ErrorLevel::warning;
		test->additionalInfo += " | Read error!";
		return TestResult(test);
	}
	if (value != address)
		test->errorLevel = ErrorLevel::warning;

	return TestResult(test);
}

#pragma endregion inherited_functions

} // namespace device
} // namespace sat