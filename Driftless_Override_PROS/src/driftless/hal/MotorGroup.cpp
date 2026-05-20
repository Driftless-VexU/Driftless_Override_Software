#include "driftless/hal/MotorGroup.hpp"

namespace driftless {
namespace hal {
void MotorGroup::addMotor(std::unique_ptr<io::IMotor>& motor) {
  m_motors.push_back(std::move(motor));
}

void MotorGroup::init() {
  for (auto& motor : m_motors)
    if (motor) motor->initialize();
}

double MotorGroup::getGearRatio() {
  double gear_ratio{};
  if (!m_motors.empty() && m_motors.front())
    gear_ratio = m_motors.front()->getGearRatio();
  return gear_ratio;
}

double MotorGroup::getAngularVelocity() {
  double average_velocity{};
  if (!m_motors.empty()) {
    for (auto& motor : m_motors)
      if (motor) average_velocity += motor->getAngularVelocity();
    average_velocity /= m_motors.size();
  }

  return average_velocity;
}

double MotorGroup::getPosition() {
  double average_position{};
  if (!m_motors.empty()) {
    for (auto& motor : m_motors)
      if (motor) average_position += motor->getPosition();
    average_position /= m_motors.size();
  }

  return average_position;
}

double MotorGroup::getEfficiency() {
  double average_efficiency{};
  if (!m_motors.empty()) {
    for (auto& motor : m_motors) {
      if (motor) {
        average_efficiency += motor->getEfficiency();
      }
    }
    average_efficiency / m_motors.size();
  }

  return average_efficiency;
}

void MotorGroup::setVoltage(double volts) {
  for (auto& motor : m_motors)
    if (motor) motor->setVoltage(volts);
}

void MotorGroup::setVelocity(double velocity) {
  for (auto& motor : m_motors)
    if (motor) motor->setVelocity(velocity);
}

void MotorGroup::setCurrentLimit(double amps) {
  for (auto& motor : m_motors) {
    if (motor) motor->setCurrentLimit(amps);
  }
}

void MotorGroup::setPosition(double position) {
  for (auto& motor : m_motors)
    if (motor) motor->setPosition(position);
}

MotorGroup& MotorGroup::operator=(MotorGroup& rhs) {
  m_motors.clear();
  for (uint8_t i{0}; i < rhs.m_motors.size(); ++i)
    m_motors.push_back(std::move(rhs.m_motors.at(i)));
  rhs.m_motors.clear();
  return *this;
}
}  // namespace hal
}  // namespace driftless