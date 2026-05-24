#include "driftless/robot/subsystems/tank_drive_train/DirectDriveBuilder.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace tank_drive_train {
DirectDriveBuilder& DirectDriveBuilder::withLeftMotor(
    std::unique_ptr<io::IMotor>& motor) {
  m_left_motors.addMotor(motor);
  return *this;
}

DirectDriveBuilder& DirectDriveBuilder::withRightMotor(
    std::unique_ptr<io::IMotor>& motor) {
  m_right_motors.addMotor(motor);
  return *this;
}

DirectDriveBuilder& DirectDriveBuilder::withVelocityToVoltage(
    double velocity_to_voltage) {
  m_velocity_to_voltage = velocity_to_voltage;
  return *this;
}
DirectDriveBuilder& DirectDriveBuilder::withWheelRadius(double wheel_radius) {
  m_wheel_radius = wheel_radius;
  return *this;
}

DirectDriveBuilder& DirectDriveBuilder::withDriveRadius(double drive_radius) {
  m_drive_radius = drive_radius;
  return *this;
}

DirectDrive DirectDriveBuilder::build() {
  return DirectDrive{std::move(*this)};
}

std::unique_ptr<DirectDrive> DirectDriveBuilder::buildUnique() {
  return std::unique_ptr<DirectDrive>{new DirectDrive{std::move(*this)}};
}
}  // namespace tank_drive_train
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless