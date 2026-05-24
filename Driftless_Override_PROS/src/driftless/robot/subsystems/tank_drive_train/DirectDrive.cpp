#include "driftless/robot/subsystems/tank_drive_train/DirectDrive.hpp"

#include "pros/screen.hpp"
namespace driftless {
namespace robot {
namespace subsystems {
namespace tank_drive_train {
void DirectDrive::init() {
  m_left_motors.init();
  m_right_motors.init();
}

void DirectDrive::run() {}

void DirectDrive::setVelocity(Velocity velocity) {
  m_left_motors.setVoltage(velocity.left_velocity * m_velocity_to_voltage);
  m_right_motors.setVoltage(velocity.right_velocity * m_velocity_to_voltage);
}

void DirectDrive::setVoltage(double left_voltage, double right_voltage) {
  m_left_motors.setVoltage(left_voltage);
  m_right_motors.setVoltage(right_voltage);
}

Velocity DirectDrive::getVelocity() {
  Velocity velocity{
      m_left_motors.getAngularVelocity() * m_wheel_radius / m_gear_ratio,
      m_right_motors.getAngularVelocity() * m_wheel_radius / m_gear_ratio};
  return velocity;
}

double DirectDrive::getDriveRadius() const { return m_drive_radius; }
}  // namespace tank_drive_train
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless