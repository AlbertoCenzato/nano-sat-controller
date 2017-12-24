#pragma once

#include "devices/DeviceI2C.hpp"
#include "devices/Interfaces.hpp"

namespace sat {

namespace utils { struct MotorSettings; }

namespace device {

class MotorActuator : public DeviceI2C, public IActuator {
public:

	// static constants
	static const std::string DEFAULT_DEV_NAME;
	static const uint8_t	 DEFAULT_I2C_ADDR = 0x40;

	static const int MOTOR_FORWARD  = 1;
	static const int MOTOR_BACKWARD = 2;
	static const int MOTOR_BRAKE    = 3;
	static const int MOTOR_RELEASE  = 4;

	int motornum = 0;
	int PWMpin = 8;
	int IN1pin = 9;
	int IN2pin = 10;

	bool running = false;
	int speed = 0;

	MotorActuator();
	MotorActuator(gnublin_i2c *bus, const utils::MotorSettings &settings);
	virtual ~MotorActuator();

	void run(int command);
	void setSpeed(int speed);

	/**
	 *	@brief This function wraps run() and setSpeed() functions for ease of use and inheritance.
	 *		   It accepts float values in [-255,255]. A negative value 'action' is equivalent to:
	 *		   motor.run(DCMotorActuator::DC_MOTOR_BACKWARD);
	 *		   setSpeed(abs(action));  // abs(x) takes the absolute value of x
	 *		   A positive 'action' value uses DC_MOTOR_FORWARD instead of DC_MOTOR_BACKWARD.
	 *		   A zero 'action' value is equivalent to:
	 *		   motor.run(DCMotorActuator::DC_MOTOR_RELEASE);
	 *	@param action: desired speed
	 */
	void act(float action) override;

	static std::unique_ptr<MotorActuator> create(gnublin_i2c *bus, const utils::MotorSettings& settings);

	TestResult testConnection() override;

	// Sends a software reset (SWRST) command to all the servo drivers on the bus
	void softwareReset(/*cls, i2c = None, i2c_bus = None*/);

protected:

	// Sets the PWM frequency
	void setPWMFreq(int freq);

	// Sets a single PWM channel"
	void setPWM(int channel, int on, int off);

	// Sets a all PWM channels
	void setAllPWM(int on, int off);
};


using MotorActuatorPtr = std::unique_ptr<MotorActuator>;

} // namespace device
} // namespace sat