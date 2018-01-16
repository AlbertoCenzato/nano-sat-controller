#pragma once

#include <chrono>

#include "devices/Interfaces.hpp"
#include "utils/DataTypes.hpp"


namespace sat {
namespace test {

/**
 * @brief 
 * Max angular speed: +-10 deg/s
 * Min angluar speed: 0 deg/s
 */
class NanoSatDynamicsMock {
   utils::Vector3f angularPosition_; // in degrees
   utils::Vector3f speed_; // in deg/s
   std::chrono::time_point<std::chrono::high_resolution_clock> lastUpdate_[3];
   const float speedGain_; // used to adjust speed to reach a maximum of 10 deg/s

public:
   explicit NanoSatDynamicsMock(utils::Vector3f startingAngularPosition);
   void setAngularVelocity(utils::Axis axis, int speed);
   utils::Vector3f getAngularPosition();
   void resetClock();

};



class MotorActuatorMock : public device::IActuator {

   const utils::Axis axis_;
   NanoSatDynamicsMock *nanoSatDynamics_;

public:

   explicit MotorActuatorMock(NanoSatDynamicsMock *dynamicsMock, utils::Axis axis);

   TestResult selfTest() noexcept override;
   bool isAvailable() const noexcept override;
   std::string getID() const noexcept override;
   std::string toString() const override;
   void act(float action) override;

};




class IMUMock {

   NanoSatDynamicsMock *nanoSatDynamics_;

public:

   explicit IMUMock(NanoSatDynamicsMock *dynamicsMock);

   TestResult selfTest() noexcept;
   bool isAvailable() const noexcept;
   std::string getID() const noexcept;
   std::string toString() const;
   std::vector<double> read() const;
   utils::Vector<float, 3> getState();
   //void startReading() override;
};
   
} // namespace test
} // namespace sat