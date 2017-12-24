/**************************************************************************/
/*! 
    @file     Adafruit_MCP9808.h
    @author   K. Townsend (Adafruit Industries)
	@license  BSD (see license.txt)
	
	This is a library for the Adafruit MCP9808 Temp Sensor breakout board
	----> http://www.adafruit.com/products/1782
	
	Adafruit invests time and resources providing this open source code, 
	please support Adafruit and open-source hardware by purchasing 
	products from Adafruit!

	@section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#pragma once

#include "Interfaces.hpp"
#include "DeviceI2C.hpp"

#pragma region registers
#define MCP9808_REG_CONFIG             0x01

#define MCP9808_REG_CONFIG_SHUTDOWN    0x0100
#define MCP9808_REG_CONFIG_CRITLOCKED  0x0080
#define MCP9808_REG_CONFIG_WINLOCKED   0x0040
#define MCP9808_REG_CONFIG_INTCLR      0x0020
#define MCP9808_REG_CONFIG_ALERTSTAT   0x0010
#define MCP9808_REG_CONFIG_ALERTCTRL   0x0008
#define MCP9808_REG_CONFIG_ALERTSEL    0x0004
#define MCP9808_REG_CONFIG_ALERTPOL    0x0002
#define MCP9808_REG_CONFIG_ALERTMODE   0x0001

#define MCP9808_REG_UPPER_TEMP         0x02
#define MCP9808_REG_LOWER_TEMP         0x03
#define MCP9808_REG_CRIT_TEMP          0x04
#define MCP9808_REG_AMBIENT_TEMP       0x05
#define MCP9808_REG_MANUF_ID           0x06
#define MCP9808_REG_DEVICE_ID          0x07

#pragma endregion registers

namespace sat {

namespace utils {	struct TemperatureSensorSettings; }

namespace device
{

class TemperatureSensor : public DeviceI2C, public ISensor {
public:

	static const std::string DEFAULT_DEV_NAME;
	static const uint8_t		 DEFAULT_I2C_ADDR = 0x18;

	TemperatureSensor();
	TemperatureSensor(gnublin_i2c *bus, const utils::TemperatureSensorSettings& settings);

	std::vector<double> read() const override;

	/**
	* @brief  Reads the 16-bit temperature register and returns the Centigrade
	*			  temperature as a float.
	*/
	float readTemperature() const;

	/**
	* @brief Set Sensor to Shutdown-State or wake up (Conf_Register BIT8)
	*			 1= shutdown / 0= wake up
	*/
	int shutdown_wake(uint8_t sw_ID);

	static std::unique_ptr<TemperatureSensor> create(gnublin_i2c *bus, const utils::TemperatureSensorSettings& settings);

	TestResult testConnection() override;
	TestResult testValues() const;

};



using TemperatureSensorPtr = std::unique_ptr<TemperatureSensor>;

} // namespace device
} // namespace sat