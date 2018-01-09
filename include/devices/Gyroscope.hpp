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

#pragma once

#include "devices/Interfaces.hpp"
#include "devices/DeviceI2C.hpp"

#include "utils/DataTypes.hpp"

namespace sat {

namespace utils { struct GyroscopeSettings; }

namespace device {

/**
 *	@brief Calss for ITG-3200 gyroscope.
 */
class Gyroscope : public DeviceI2C, public ISensor {

public:

	static const std::string DEFAULT_DEV_ID;
	static const uint8_t		 DEFAULT_I2C_ADDR = 0x68;

	// "The LSB bit of the 7 bit address is determined by the logic level on pin 9. 
	// This allows two ITG-3200 devices to be connected to the same I2C bus.
	// One device should have pin9 (or bit0) LOW and the other should be HIGH." source: ITG3200 datasheet
	// Note that pin9 (AD0 - I2C Slave Address LSB) may not be available on some breakout boards so check 
	// the schematics of your breakout board for the correct address to use.
	static const uint8_t ITG3200_ADDR_AD0_LOW  = 0x68; //AD0=0 0x68 I2C address when AD0 is connected to LOW (GND)
	static const uint8_t ITG3200_ADDR_AD0_HIGH = 0x69; //AD0=1 0x69 I2C address when AD0 is connected to HIGH (VCC) - default for sparkfun breakout


   /**
    * @brief best way to instantiate a Gyroscope object
    * 
    * Static function that returns a managed pointer to a Gyroscope object.
    * See this class' constructors for futher details.
    */
   static std::unique_ptr<Gyroscope> create(const utils::GyroscopeSettings& settings);

   /**
    * @brief default constructor
    */
	Gyroscope();

   /**
    * @brief class constructor
    */
	Gyroscope(const utils::GyroscopeSettings& settings);

   
   /**
	 *	@brief Reads the angular rate (deg/s) of the device
	 */
	std::vector<double> read() const override;

   /**
    * @brief Reads the angular rate (deg/s) of the device
    */
	utils::Vector3f readGyro() const; // includes gain and offset

   /**
	 *	@brief Reading function, does not apply gains and offsets
	 *	@return Raw readings for X, Y, Z axis in this order
	 */
	utils::Vector3f readGyroRaw() const;

   /**
	 *	@brief Tests if the device is connected
	 */
   TestResult testConnection() override;
   
   /**
    * @retrun offsets by which each measure is translated 
    *         before being multiplyied by the gains
    */
   utils::Vector3f getOffsets() const;

   /**
    * @return gains applyied to every measurement
    */
   utils::Vector3f getGains() const;

   /**
    * @param offsets
    */
   void setOffsets(const utils::Vector3f& offsets);

   /**
    * @param gains
    */
	void setGains(const utils::Vector3f& gains);

   /**
    * @brief calibrates offests to have 0 deg/s on every axis when gyro is not moving
    * 
    * @param totSamples number of measures to do
    * @param sampleDelay waiting time between two measures
    * 
    * @note do not move the gyroscope during calibration 
    */
   template<typename _Rep, typename _Period>
	void zeroCalibrate(int totSamples, std::chrono::duration<_Rep,_Period> sampleDelay) {
      utils::Vector3f tmpOffsets{ 0.f,0.f,0.f };
      for (auto i = 0; i < totSamples; i++) {
         this_thread::sleep_for(sampleDelay);
         tmpOffsets += readGyroRaw();
      }

      setOffsets(tmpOffsets / float(totSamples));
   }
	

	// Who Am I
	void setDevAddr(unsigned int _addr);

	// Sample Rate Divider
	uint8_t getSampleRateDiv() const;
	void setSampleRateDiv(uint8_t _SampleRate);

	// Digital Low Pass Filter BandWidth and SampleRate 
	uint8_t getFSRange() const;
	void setFSRange(uint8_t _Range); // RANGE2000

	uint8_t getFilterBW() const;
	void setFilterBW(uint8_t _BW); // see register parameters above

	// Interrupt Configuration
	bool isINTActiveOnLow() const;
	void setINTLogiclvl(bool _State); //ACTIVE_ONHIGH, ACTIVE_ONLOW

	// Interrupt drive type
	bool isINTOpenDrain() const;
	void setINTDriveType(bool _State); //OPEN_DRAIN, PUSH_PULL

	// Interrupt Latch mode
	bool isLatchUntilCleared() const;
	void setLatchMode(bool _State); //UNTIL_INT_CLEARED, PULSE_50US

	// Interrupt Latch clear method
	bool isAnyRegClrMode() const;
	void setLatchClearMode(bool _State); //READ_ANYREG, READ_STATUSREG

	// INT pin triggers
	bool isITGReadyOn() const;
	void setITGReady(bool _State);

	bool isRawDataReadyOn() const;
	void setRawDataReady(bool _State);
	
	// Trigger Status
	bool isITGReady() const;
	bool isRawDataReady() const;
	
	// Gyro Sensors
	void readTemp(float *_Temp) const;
	
	// Power management
	void reset(); // after reset all registers have default values
	bool isLowPower() const;
	void setPowerMode(bool _State); // NORMAL, STANDBY
	bool isXgyroStandby() const;
	bool isYgyroStandby() const;
	bool isZgyroStandby() const;
	void setXgyroStandby(bool _Status); // NORMAL, STANDBY
	void setYgyroStandby(bool _Status);
	void setZgyroStandby(bool _Status);
	uint8_t getClockSource() const;
	void setClockSource(uint8_t _CLKsource); // see register parameters above

protected:
   utils::Vector3f gains_;
   utils::Vector3f offsets_;
};


using GyroscopeSensorPtr = std::unique_ptr<Gyroscope>;


} // namespace device
} // namespace sat