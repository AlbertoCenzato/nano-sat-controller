#pragma once

#include "devices/Interfaces.hpp"
#include "devices/DeviceI2C.hpp"

namespace sat {

namespace utils { struct PressureSensorSettings; }

namespace device {

class PressureSensor : public DeviceI2C, public ISensor {

public:

	static const std::string DEFAULT_DEV_NAME;
	static const uint8_t		 DEFAULT_I2C_ADDR = 0x77;  // default I2C address of BMP085
	static const unsigned char OSS = 0;  // Oversampling Setting

	// Calibration values
	uint16_t ac1 = 0;	// WARNING originally int, not uint
	uint16_t ac2 = 0;	// WARNING originally int, not uint
	uint16_t ac3 = 0;	// WARNING originally int, not uint
	uint16_t ac4 = 0;
	uint16_t ac5 = 0;
	uint16_t ac6 = 0;
	uint16_t b1  = 0;  // WARNING originally int, not uint
	uint16_t b2  = 0;  // WARNING originally int, not uint
	uint16_t mb  = 0;  // WARNING originally int, not uint
	uint16_t mc  = 0;  // WARNING originally int, not uint
	uint16_t md  = 0;  // WARNING originally int, not uint

	PressureSensor();
	PressureSensor(gnublin_i2c *bus, const utils::PressureSensorSettings& settings);
	~PressureSensor();

	std::vector<double> read() const override;

	long readPressure() const;
	long readPressureRaw() const;

	static std::unique_ptr<PressureSensor> create(gnublin_i2c *bus, const utils::PressureSensorSettings& settings);

	TestResult testConnection() override;

private:

	/**
	* @brief Calculate temperature given ut.
	*			 Value returned will be in units of 0.1 deg C
	*/
	long get_temperature(unsigned int ut) const;

	/**
	* @brief Calculate pressure given up. Calibration values must be known.
	*			 b5 is also required so bmp085GetTemperature(...) must be called first.
	*			 Value returned will be pressure in units of Pa.
	*/
	long get_pressure(unsigned long up) const;
	
	/**
	 *	@brief Read the uncompensated temperature value
	 */
	unsigned int readUT() const;

	/**
	 *	@brief Read the uncompensated pressure value
	 */
	unsigned long readUP() const;

};


using PressureSensorPtr = std::unique_ptr<PressureSensor>;

} // namespace device
} // namespace sat