#include "driftless/robot/subsystems/tank_drive_train/TankDriveTrainSubsystem.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace tank_drive_train {
TankDriveTrainSubsystem::TankDriveTrainSubsystem(
    std::unique_ptr<ITankDriveTrain>& drivetrain)
    : ASubsystem{ESubsystem::TANK_DRIVE_TRAIN}, m_drive_train(std::move(drivetrain)) {}

void TankDriveTrainSubsystem::init() { m_drive_train->init(); }

void TankDriveTrainSubsystem::run() { m_drive_train->run(); }

void TankDriveTrainSubsystem::command(const commands::Command& cmd) {
  std::visit([this](auto&& cmd) {this->handleCommand(cmd);}, cmd);
}

void* TankDriveTrainSubsystem::state(ESubsystemState state_name) {
  void* result{nullptr};

  if (state_name == ESubsystemState::DRIVETRAIN_GET_VELOCITY) {
    Velocity* velocity{new Velocity{m_drive_train->getVelocity()}};
    result = velocity;
  } else if (state_name == ESubsystemState::DRIVETRAIN_GET_RADIUS) {
    double* radius{new double{m_drive_train->getDriveRadius()}};
    result = radius;
  }
  return result;
}

void TankDriveTrainSubsystem::handleCommand(const commands::tank_drive_train::SetVelocityCommand& cmd) {
  m_drive_train->setVelocity({cmd.m_left_velocity, cmd.m_right_velocity});
}

void TankDriveTrainSubsystem::handleCommand(const commands::tank_drive_train::SetVoltageCommand& cmd) {
  m_drive_train->setVoltage(cmd.m_left_voltage, cmd.m_right_voltage);
}
}  // namespace drivetrain
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless