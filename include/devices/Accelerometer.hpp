/*===========================================================================
								Nano satellite controller

// Copyright: Copyright (c) 2017, Alberto Cenzato
					  All rights reserved.

// Licence: GNU

// Based on: ADXL345 Driver for Arduino 
			 https://github.com/jenschr/Arduino-libraries/tree/master/ADXL345

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU License.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU License V2 for more details: https://www.gnu.org/licenses/gpl-2.0.html

//============================================================================ */

#pragma once

#include "devices/DeviceI2C.hpp"
#include "devices/Interfaces.hpp"

#include "utils/DataTypes.hpp"
#include "Accelerometer.hpp"

namespace sat {

// forward declaration
namespace utils {	struct AccelerometerSettigs; }

namespace device {


/**
 *	@brief Class for the ADXL345 accelerometer sensor
 */
class Accelerometer : public DeviceI2C, public ISensor {

public:

	static const std::string DEFAULT_DEV_NAME;
	static const uint8_t	 DEFAULT_I2C_ADDR = 0x1D;

	static const uint8_t ADXL345_ADDR_ALT_HIGH = 0x1D; // Accelerometer address when ALT is connected to HIGH
	static const uint8_t ADXL345_ADDR_ALT_LOW  = 0x53; // Accelerometer address when ALT is connected to LOW

	bool	  status;        // set when error occurs see error code for details
	uint8_t error_code;    // Initial state

	Accelerometer();
	Accelerometer(gnublin_i2c *bus, const utils::AccelerometerSettigs& settings);

	/**
	 *	@brief Generic reading function, overrides ISensor::read()
	 *	@return vector of size = 3 with readings (in milli-g) for X, Y, Z axis in this order
	 */
	std::vector<double> read() const override;

	/**
	 *	@brief Reading function
	 *	@return Readings (in milli-g) for X, Y, Z axis in this order
	 */
	utils::Vector3f readAccel() const;

	/**
	 *	@brief Reading function
	 *	@return Raw readings for X, Y, Z axis in this order
	 */
	utils::Vector3f readAccelRaw() const;

	/**
	 *	@brief Tests if the device is connected
	 */
	TestResult testConnection() override;

	void setTapThreshold(int tapThreshold);
	int  getTapThreshold() const;

   utils::Vector3f getOffsets() const;
   utils::Vector3f getGains() const;

   void setOffsets(const utils::Vector3f& offsets);
	void setGains(const utils::Vector3f& gains);

	/**
	* @brief Sets the OFSX, OFSY and OFSZ bytes
	*		  OFSX, OFSY and OFSZ are user offset adjustments in twos complement format with
	*		  a scale factor of 15,6mg/LSB
	*		  OFSX, OFSY and OFSZ should be comprised between
	*/
	//void setOffsets(const utils::Vector<std::uint8_t, 3>& offsets);
   //utils::Vector3i getOffsets() const;

	void setTapDuration(int tapDuration);
	int  getTapDuration() const;

	void setDoubleTapLatency(int floatTapLatency);
	int  getDoubleTapLatency() const;

	void setDoubleTapWindow(int floatTapWindow);
	int  getDoubleTapWindow() const;

	void setActivityThreshold(int activityThreshold);
	int  getActivityThreshold() const;

	void setInactivityThreshold(int inactivityThreshold);
	int  getInactivityThreshold() const;

	void setTimeInactivity(int timeInactivity);
	int  getTimeInactivity() const;

	void setFreeFallThreshold(int freeFallthreshold);
	int  getFreeFallThreshold() const;

	void setFreeFallDuration(int freeFallDuration);
	int  getFreeFallDuration() const;

	bool isActivityXEnabled() const;
	bool isActivityYEnabled() const;
	bool isActivityZEnabled() const;
	bool isInactivityXEnabled() const;
	bool isInactivityYEnabled() const;
	bool isInactivityZEnabled() const;
	bool isActivityAc() const;
	bool isInactivityAc() const;
	void setActivityAc(bool state);
	void setInactivityAc(bool state);

	void setSuppressBit(bool state);
	bool getSuppressBit() const;

	bool isTapDetectionOnX() const;
	void setTapDetectionOnX(bool state);
	bool isTapDetectionOnY() const;
	void setTapDetectionOnY(bool state);
	bool isTapDetectionOnZ() const;
	void setTapDetectionOnZ(bool state);

	void setActivityX(bool state);
	void setActivityY(bool state);
	void setActivityZ(bool state);
	void setInactivityX(bool state);
	void setInactivityY(bool state);
	void setInactivityZ(bool state);

	bool isActivitySourceOnX() const;
	bool isActivitySourceOnY() const;
	bool isActivitySourceOnZ() const;
	bool isTapSourceOnX() const;
	bool isTapSourceOnY() const;
	bool isTapSourceOnZ() const;
	bool isAsleep() const;

	bool isLowPower() const;
	void setLowPower(bool state);

	void  setRate(float rate);
	float getRate() const;

	void    set_bw(uint8_t bw_code);
	uint8_t get_bw_code() const;

	uint8_t getInterruptSource() const;
	bool getInterruptSource(uint8_t interruptBit) const;
	bool getInterruptMapping(uint8_t interruptBit) const;
	void setInterruptMapping(uint8_t interruptBit, bool interruptPin);
	bool isInterruptEnabled(uint8_t interruptBit) const;
	void setInterrupt(uint8_t interruptBit, bool state);

	void getRangeSetting(uint8_t* rangeSetting) const;
	void setRangeSetting(int val);
	bool getSelfTestBit() const;
	void setSelfTestBit(bool selfTestBit);
	bool getSpiBit() const;
	void setSpiBit(bool spiBit);
	bool getInterruptLevelBit() const;
	void setInterruptLevelBit(bool interruptLevelBit);
	bool getFullResBit() const;
	void setFullResBit(bool fullResBit);
	bool getJustifyBit() const;
	void setJustifyBit(bool justifyBit);

   template<typename _Rep, typename _Period>
   void zeroCalibrate(int totSamples, std::chrono::duration<_Rep,_Period> sampleDelay, utils::Axis verticalAxis) {
      
      utils::Vector3f tmp{ 0.f, 0.f, 0.f };
      for (auto i = 0; i < totSamples; i++) {
         tmp += readAccelRaw();
         this_thread::sleep_for(sampleDelay);
      }

      tmp = tmp / float(totSamples);
      tmp[int(verticalAxis)] -= 1000.f/gains_[int(verticalAxis)];

      offsets_ = tmp;
      std::cout << "Offsets: " << offsets_ << std::endl;
   }

	static std::unique_ptr<Accelerometer> create(gnublin_i2c *bus, const utils::AccelerometerSettigs& settings);

protected:

   utils::Vector3f gains_;  // counts to Gs
   utils::Vector3f offsets_;

	void retrieveSettings(const utils::AccelerometerSettigs& settings);

	void setRegisterBit(uint8_t regAdress, int bitPos, bool state);
	bool getRegisterBit(uint8_t regAdress, int bitPos) const;
	
};

//void print_byte(uint8_t val);

using AccelerometerSensorPtr = std::unique_ptr<Accelerometer>;

} // namespace device
} // namespace sat