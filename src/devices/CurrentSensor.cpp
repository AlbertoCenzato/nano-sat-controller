/*===========================================================================
									Nano satellite controller

// Copyright: Copyright (c) 2017, Alberto Cenzato
				  All rights reserved.

// Licence: BSD

// Based on: Adafruit INA219 breakout board library
				 by K. Townsend (Adafruit Industries)
				 https://github.com/adafruit/Adafruit_INA219

Redistribution and use in source and binary forms are permitted provided that
the above copyright notice and this paragraph are duplicated in all such
forms and that any documentation, advertising materials, and other materials
related to such distribution and use acknowledge that the software was developed
by the <organization>. The name of the <organization> may not be used to endorse
or promote products derived from this software without specific prior written
permission.
THIS SOFTWARE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTIES OF MERCHANTABILITY
AND FITNESS FOR A PARTICULAR PURPOSE.

//============================================================================ */

#include "devices/CurrentSensor.hpp"

#include <thread>

#include "utils/Settings.hpp"
#include "self_test/SelfTest.hpp"
#include "utils/Logger.hpp"


#pragma region registers

#define INA219_READ 0x01

//CONFIG REGISTER (R/W)
#define INA219_REG_CONFIG 0x00

#define INA219_CONFIG_RESET					0x8000  // Reset 
#define INA219_CONFIG_BVOLTAGERANGE_MASK	0x2000  // Bus Voltage Range Mask
#define INA219_CONFIG_BVOLTAGERANGE_16V	0x0000  // 0-16V Range
#define INA219_CONFIG_BVOLTAGERANGE_32V	0x2000  // 0-32V 
#define INA219_CONFIG_GAIN_MASK				0x1800  // Gain Mask
#define INA219_CONFIG_GAIN_1_40MV			0x0000  // Gain 1, 40mV Range
#define INA219_CONFIG_GAIN_2_80MV			0x0800  // Gain 2, 80mV Range
#define INA219_CONFIG_GAIN_4_160MV			0x1000  // Gain 4, 160mV Range
#define INA219_CONFIG_GAIN_8_320MV			0x1800  // Gain 8, 320mV 
#define INA219_CONFIG_BADCRES_MASK			0x0780  // Bus ADC Resolution Mask
#define INA219_CONFIG_BADCRES_9BIT			0x0080  // 9-bit bus res = 0..511
#define INA219_CONFIG_BADCRES_10BIT			0x0100  // 10-bit bus res = 0..1023
#define INA219_CONFIG_BADCRES_11BIT			0x0200  // 11-bit bus res = 0..2047
#define INA219_CONFIG_BADCRES_12BIT			0x0400  // 12-bit bus res = 0..

#define INA219_CONFIG_SADCRES_MASK					0x0078  // Shunt ADC Resolution and Averaging Mask
#define INA219_CONFIG_SADCRES_9BIT_1S_84US		0x0000  // 1 x 9-bit shunt sample
#define INA219_CONFIG_SADCRES_10BIT_1S_148US		0x0008  // 1 x 10-bit shunt sample
#define INA219_CONFIG_SADCRES_11BIT_1S_276US		0x0010  // 1 x 11-bit shunt sample
#define INA219_CONFIG_SADCRES_12BIT_1S_532US		0x0018  // 1 x 12-bit shunt sample
#define INA219_CONFIG_SADCRES_12BIT_2S_1060US	0x0048	// 2 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_4S_2130US	0x0050  // 4 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_8S_4260US	0x0058  // 8 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_16S_8510US	0x0060  // 16 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_32S_17MS		0x0068  // 32 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_64S_34MS		0x0070  // 64 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_128S_69MS	0x0078  // 128 x 12-bit shunt samples averaged 

#define INA219_CONFIG_MODE_MASK						0x0007  // Operating Mode Mask
#define INA219_CONFIG_MODE_POWERDOWN				0x0000
#define INA219_CONFIG_MODE_SVOLT_TRIGGERED		0x0001
#define INA219_CONFIG_MODE_BVOLT_TRIGGERED		0x0002
#define INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED	0x0003
#define INA219_CONFIG_MODE_ADCOFF					0x0004
#define INA219_CONFIG_MODE_SVOLT_CONTINUOUS		0x0005
#define INA219_CONFIG_MODE_BVOLT_CONTINUOUS		0x0006
#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS 0x0007

// SHUNT VOLTAGE REGISTER (R)
#define INA219_REG_SHUNTVOLTAGE 0x01

// BUS VOLTAGE REGISTER (R)
#define INA219_REG_BUSVOLTAGE 0x02

// POWER REGISTER (R)
#define INA219_REG_POWER 0x03

// CURRENT REGISTER (R)
#define INA219_REG_CURRENT 0x04

// CALIBRATION REGISTER (R/W)
#define INA219_REG_CALIBRATION 0x05

#pragma endregion registers


using this_thread::sleep_for;

namespace sat {
namespace device
{

const string CurrentSensor::DEFAULT_DEV_ID = "Adafruit_INA219";
const chrono::seconds CurrentSensor::DEVICE_RESET_TIME = chrono::seconds(1);

// ---------- Constructors ----------
#pragma region constructors

std::unique_ptr<CurrentSensor> CurrentSensor::create(const utils::CurrentSensorSettings& settings) {
	return std::make_unique<CurrentSensor>(settings);
}

CurrentSensor::CurrentSensor() : DeviceI2C(DEFAULT_DEV_ID, DEFAULT_I2C_ADDR), ina219_calValue(0), 
     ina219_currentDivider_mA(0), ina219_powerDivider_mW(0) { }

CurrentSensor::CurrentSensor(const utils::CurrentSensorSettings& settings) 
   : DeviceI2C(settings.deviceID, settings.address), ina219_calValue(0), 
     ina219_currentDivider_mA(0), ina219_powerDivider_mW(0) 
{
   addTestAndAction(&CurrentSensor::testValues, &CurrentSensor::noAction);
	//testsAndActions.emplace_back(std::bind(&CurrentSensor::testValues, this),
	//									  std::bind(&CurrentSensor::noAction, this));
	setCalibration_32V_2A();
}

#pragma endregion constructors

// ---------- public member functions ------------------
#pragma region pub_functions

float CurrentSensor::readCurrent() const {
	if (!available)
		return std::numeric_limits<float>::quiet_NaN();

	uint16_t value;

	// Sometimes a sharp load will reset the INA219, which will
	// reset the cal register, meaning CURRENT and POWER will
	// not be available ... avoid this by always setting a cal
	// value even if it's an unfortunate extra step
	write16_const(INA219_REG_CALIBRATION, ina219_calValue);

	// Now we can safely read the CURRENT register!
	read16(INA219_REG_CURRENT, value);

	float valueDec = int16_t(value);
	valueDec /= float(ina219_currentDivider_mA);
	return valueDec;
}

vector<double> CurrentSensor::read() const {
	return { double(readCurrent()) };
}


void CurrentSensor::setCalibration_32V_2A() {
	// By default we use a pretty huge range for the input voltage,
	// which probably isn't the most appropriate choice for system
	// that don't use a lot of power.  But all of the calculations
	// are shown below if you want to change the settings.  You will
	// also need to change any relevant register settings, such as
	// setting the VBUS_MAX to 16V instead of 32V, etc.

	// VBUS_MAX = 32V             (Assumes 32V, can also be set to 16V)
	// VSHUNT_MAX = 0.32          (Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
	// RSHUNT = 0.1               (Resistor value in ohms)

	// 1. Determine max possible current
	// MaxPossible_I = VSHUNT_MAX / RSHUNT
	// MaxPossible_I = 3.2A

	// 2. Determine max expected current
	// MaxExpected_I = 2.0A

	// 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
	// MinimumLSB = MaxExpected_I/32767
	// MinimumLSB = 0.000061              (61uA per bit)
	// MaximumLSB = MaxExpected_I/4096
	// MaximumLSB = 0,000488              (488uA per bit)

	// 4. Choose an LSB between the min and max values
	//    (Preferrably a roundish number close to MinLSB)
	// CurrentLSB = 0.0001 (100uA per bit)

	// 5. Compute the calibration register
	// Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
	// Cal = 4096 (0x1000)

	ina219_calValue = 4096;

	// 6. Calculate the power LSB
	// PowerLSB = 20 * CurrentLSB
	// PowerLSB = 0.002 (2mW per bit)

	// 7. Compute the maximum current and shunt voltage values before overflow
	//
	// Max_Current = Current_LSB * 32767
	// Max_Current = 3.2767A before overflow
	//
	// If Max_Current > Max_Possible_I then
	//    Max_Current_Before_Overflow = MaxPossible_I
	// Else
	//    Max_Current_Before_Overflow = Max_Current
	// End If
	//
	// Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
	// Max_ShuntVoltage = 0.32V
	//
	// If Max_ShuntVoltage >= VSHUNT_MAX
	//    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
	// Else
	//    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
	// End If

	// 8. Compute the Maximum Power
	// MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
	// MaximumPower = 3.2 * 32V
	// MaximumPower = 102.4W

	// Set multipliers to convert raw current/power values
	ina219_currentDivider_mA = 10;  // Current LSB = 100uA per bit (1000/100 = 10)
	ina219_powerDivider_mW = 2;     // Power LSB = 1mW per bit (2/1)

											  // Set Calibration register to 'Cal' calculated above	
	write16(INA219_REG_CALIBRATION, ina219_calValue);

	// Set Config register to take into account the settings above
	const uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
							      INA219_CONFIG_GAIN_8_320MV |
							      INA219_CONFIG_BADCRES_12BIT |
							      INA219_CONFIG_SADCRES_12BIT_1S_532US |
							      INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

	write16(INA219_REG_CONFIG, config);
}


void CurrentSensor::setCalibration_32V_1A() {
	// By default we use a pretty huge range for the input voltage,
	// which probably isn't the most appropriate choice for system
	// that don't use a lot of power.  But all of the calculations
	// are shown below if you want to change the settings.  You will
	// also need to change any relevant register settings, such as
	// setting the VBUS_MAX to 16V instead of 32V, etc.

	// VBUS_MAX = 32V		(Assumes 32V, can also be set to 16V)
	// VSHUNT_MAX = 0.32	(Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
	// RSHUNT = 0.1			(Resistor value in ohms)

	// 1. Determine max possible current
	// MaxPossible_I = VSHUNT_MAX / RSHUNT
	// MaxPossible_I = 3.2A

	// 2. Determine max expected current
	// MaxExpected_I = 1.0A

	// 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
	// MinimumLSB = MaxExpected_I/32767
	// MinimumLSB = 0.0000305             (30.5�A per bit)
	// MaximumLSB = MaxExpected_I/4096
	// MaximumLSB = 0.000244              (244�A per bit)

	// 4. Choose an LSB between the min and max values
	//    (Preferrably a roundish number close to MinLSB)
	// CurrentLSB = 0.0000400 (40�A per bit)

	// 5. Compute the calibration register
	// Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
	// Cal = 10240 (0x2800)

	ina219_calValue = 10240;

	// 6. Calculate the power LSB
	// PowerLSB = 20 * CurrentLSB
	// PowerLSB = 0.0008 (800�W per bit)

	// 7. Compute the maximum current and shunt voltage values before overflow
	//
	// Max_Current = Current_LSB * 32767
	// Max_Current = 1.31068A before overflow
	//
	// If Max_Current > Max_Possible_I then
	//    Max_Current_Before_Overflow = MaxPossible_I
	// Else
	//    Max_Current_Before_Overflow = Max_Current
	// End If
	//
	// ... In this case, we're good though since Max_Current is less than MaxPossible_I
	//
	// Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
	// Max_ShuntVoltage = 0.131068V
	//
	// If Max_ShuntVoltage >= VSHUNT_MAX
	//    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
	// Else
	//    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
	// End If

	// 8. Compute the Maximum Power
	// MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
	// MaximumPower = 1.31068 * 32V
	// MaximumPower = 41.94176W

	// Set multipliers to convert raw current/power values
	ina219_currentDivider_mA = 25;      // Current LSB = 40uA per bit (1000/40 = 25)
	ina219_powerDivider_mW = 1;         // Power LSB = 800�W per bit

													// Set Calibration register to 'Cal' calculated above	
	write16(INA219_REG_CALIBRATION, ina219_calValue);

	// Set Config register to take into account the settings above
	const uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
							      INA219_CONFIG_GAIN_8_320MV |
							      INA219_CONFIG_BADCRES_12BIT |
							      INA219_CONFIG_SADCRES_12BIT_1S_532US |
							      INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

	write16(INA219_REG_CONFIG, config);
}

void CurrentSensor::setCalibration_16V_400mA() {

	// Calibration which uses the highest precision for 
	// current measurement (0.1mA), at the expense of 
	// only supporting 16V at 400mA max.

	// VBUS_MAX = 16V
	// VSHUNT_MAX = 0.04          (Assumes Gain 1, 40mV)
	// RSHUNT = 0.1               (Resistor value in ohms)

	// 1. Determine max possible current
	// MaxPossible_I = VSHUNT_MAX / RSHUNT
	// MaxPossible_I = 0.4A

	// 2. Determine max expected current
	// MaxExpected_I = 0.4A

	// 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
	// MinimumLSB = MaxExpected_I/32767
	// MinimumLSB = 0.0000122              (12uA per bit)
	// MaximumLSB = MaxExpected_I/4096
	// MaximumLSB = 0.0000977              (98uA per bit)

	// 4. Choose an LSB between the min and max values
	//    (Preferrably a roundish number close to MinLSB)
	// CurrentLSB = 0.00005 (50uA per bit)

	// 5. Compute the calibration register
	// Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
	// Cal = 8192 (0x2000)

	ina219_calValue = 8192;

	// 6. Calculate the power LSB
	// PowerLSB = 20 * CurrentLSB
	// PowerLSB = 0.001 (1mW per bit)

	// 7. Compute the maximum current and shunt voltage values before overflow
	//
	// Max_Current = Current_LSB * 32767
	// Max_Current = 1.63835A before overflow
	//
	// If Max_Current > Max_Possible_I then
	//    Max_Current_Before_Overflow = MaxPossible_I
	// Else
	//    Max_Current_Before_Overflow = Max_Current
	// End If
	//
	// Max_Current_Before_Overflow = MaxPossible_I
	// Max_Current_Before_Overflow = 0.4
	//
	// Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
	// Max_ShuntVoltage = 0.04V
	//
	// If Max_ShuntVoltage >= VSHUNT_MAX
	//    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
	// Else
	//    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
	// End If
	//
	// Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
	// Max_ShuntVoltage_Before_Overflow = 0.04V

	// 8. Compute the Maximum Power
	// MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
	// MaximumPower = 0.4 * 16V
	// MaximumPower = 6.4W

	// Set multipliers to convert raw current/power values
	ina219_currentDivider_mA = 20;  // Current LSB = 50uA per bit (1000/50 = 20)
	ina219_powerDivider_mW = 1;     // Power LSB = 1mW per bit

											  // Set Calibration register to 'Cal' calculated above 
	write16(INA219_REG_CALIBRATION, ina219_calValue);

	// Set Config register to take into account the settings above
	const uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
		                     INA219_CONFIG_GAIN_1_40MV |
		                     INA219_CONFIG_BADCRES_12BIT |
		                     INA219_CONFIG_SADCRES_12BIT_1S_532US |
		                     INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

	write16(INA219_REG_CONFIG, config);
}


float CurrentSensor::getShuntVoltage_mV() const {
	
	uint16_t value_raw;
	read16(INA219_REG_SHUNTVOLTAGE, value_raw);

	return int16_t(value_raw) * 0.01f;
}


float CurrentSensor::getBusVoltage_V() const {
	uint16_t value_raw;
	read16(INA219_REG_BUSVOLTAGE, value_raw);

	// Shift to the right 3 to drop CNVR and OVF and multiply by LSB
	const auto value = int16_t((value_raw >> 3) * 4);
	return value * 0.001f;
}

#pragma endregion pub_functions



// ---------- Inherited functions ----------
#pragma region inherited_functions


bool CurrentSensor::read16(uint8_t regAddress, uint16_t& value) const {

	bus.setAddress(address);
	if (bus.send(regAddress) == -1) {
		Log::err << deviceID + ": " + bus.getErrorMessage();
		return false;
	}

	sleep_for(chrono::microseconds(600));

	uint8_t readBuff[2];
	if (bus.receive(readBuff, 2) == -1) {
		Log::err << deviceID + ": " + bus.getErrorMessage();
		return false;
	}

	value = static_cast<uint16_t>(readBuff[0] << 8 | readBuff[1]);

	return true;
}


TestResult CurrentSensor::testConnection() {
	auto test = ConnectionSelfTest::create(deviceID);
	uint16_t value, savedState;
	if (!read16(INA219_REG_CONFIG, savedState)) {	// save value stored in INA219_REG_CONFIG register
		test->errorLevel = ErrorLevel::warning;
		test->additionalInfo += " | Read error!";
		return TestResult(test);
	}
	if (!write16(INA219_REG_CONFIG, INA219_CONFIG_RESET)) { // setting RESET bit to 1
		test->errorLevel = ErrorLevel::warning;
		test->additionalInfo += " | Read error!";
		return TestResult(test);
	}

	sleep_for(DEVICE_RESET_TIME); // wait 1s for device reset

	if (!read16(INA219_REG_CONFIG, value)) {	// read default value of INA219_REG_CONFIG register
		test->errorLevel = ErrorLevel::warning;
		test->additionalInfo += " | Read error!";
		return TestResult(test);
	}
	if (value != 0x399F)
		test->errorLevel = ErrorLevel::warning;

	if (!write16(INA219_REG_CONFIG, savedState)) {	// restore saved register value
		test->errorLevel = ErrorLevel::warning;
		test->additionalInfo += " | Read error!";
		return TestResult(test);
	}

	return TestResult(test);
}

TestResult CurrentSensor::testValues() const {

	auto test = OutOfRangeSelfTest::create(deviceID);
	const auto current = readCurrent();
	if (current < -2000 || current > 2000) {
		test->errorLevel = ErrorLevel::warning;
	}
	test->additionalInfo += " | current value (mA): " + to_string(current);

	return TestResult(test);
}
#pragma endregion inherited_functions

	} // end device
} // end sat