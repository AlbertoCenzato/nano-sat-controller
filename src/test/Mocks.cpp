#include "test/Mocks.hpp"
#include "utils/DataTypes.hpp"

namespace sat {
namespace test {

// ------------- NanoSatDynamicsMock ------------------

NanoSatDynamicsMock::NanoSatDynamicsMock(utils::Vector3f startingAngularPosition) 
   : angularPosition_(startingAngularPosition), speed_{ 0.f, 0.f, 0.f }, speedGain_(10/255) {
   
   for (auto &time : lastUpdate_)
      time = std::chrono::high_resolution_clock::now();
}

void NanoSatDynamicsMock::setAngularVelocity(utils::Axis axis, int speed) {
   const auto index = static_cast<int>(axis);
   const auto now = std::chrono::high_resolution_clock::now();

   // elapsed time in seconds
   const auto dt = std::chrono::duration_cast<std::chrono::microseconds>(now - lastUpdate_[index]).count() / 10E6;
   angularPosition_[index] += speed_[index] * dt; // update angularPosition only on one axis to keep tack of 

   if (angularPosition_[index] >= 360.f)   angularPosition_[index] -= 360.f;
   else if (angularPosition_[index] < 0.f) angularPosition_[index] += 360.f;

   lastUpdate_[index] = now;
   speed_[index] = speedGain_ * speed; // set new speed
}

utils::Vector3f NanoSatDynamicsMock::getAngularPosition() {
   const auto now = std::chrono::high_resolution_clock::now();

   // elapsed time in seconds
   for (auto axis = 0; axis < 3; ++axis) {
      const auto dt = std::chrono::duration_cast<std::chrono::microseconds>(now - lastUpdate_[axis]).count() / 10E6;
      angularPosition_[axis] += speed_[axis] * dt; // update angularPosition on all axes
   }

   for (auto &angle : angularPosition_) {
      if (angle >= 360.f)   angle -= 360.f;
      else if (angle < 0.f) angle += 360.f;
   }

   for (auto& time : lastUpdate_)
      time = now;

   return angularPosition_;
}

void NanoSatDynamicsMock::resetClock() {
   for (auto& clock : lastUpdate_)
      clock = std::chrono::high_resolution_clock::now();
}


// ------------- MotorActuatorMock ------------------

MotorActuatorMock::MotorActuatorMock(NanoSatDynamicsMock* dynamicsMock, utils::Axis axis)
   : axis_(axis), nanoSatDynamics_(dynamicsMock) { }

TestResult MotorActuatorMock::selfTest() noexcept { return TestResult(); }

bool MotorActuatorMock::isAvailable() const noexcept { return true; }

std::string MotorActuatorMock::getID() const noexcept { return "MotorActuatorMock"; }

std::string MotorActuatorMock::toString() const { return "MotorActuatorMock"; }

void MotorActuatorMock::act(float action) {
   if (action > 255)  action = 255;
   if (action < -255) action = -255;
   nanoSatDynamics_->setAngularVelocity(axis_, action);
}



// ------------- IMUMock ------------------

IMUMock::IMUMock(NanoSatDynamicsMock* dynamicsMock) : nanoSatDynamics_(dynamicsMock) { }

TestResult IMUMock::selfTest() noexcept { return TestResult(); }

bool IMUMock::isAvailable() const noexcept { return true; }

std::string IMUMock::getID() const noexcept { return "IMUMock"; }

std::string IMUMock::toString() const { return "IMUMock"; }

std::vector<double> IMUMock::read() const {
   return std::vector<double>{0.,0.,0.};
}

utils::Vector<float, 3> IMUMock::getState() {
   return nanoSatDynamics_->getAngularPosition();
}

//void IMUMock::startReading() {
//   nanoSatDynamics_->resetClock();
//}


}  // namespace test
} // namespace sat