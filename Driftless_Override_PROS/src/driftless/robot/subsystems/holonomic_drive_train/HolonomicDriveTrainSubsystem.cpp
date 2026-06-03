#include "driftless/robot/subsystems/holonomic_drive_train/HolonomicDriveTrainSubsystem.hpp"

namespace driftless::robot::subsystems::holonomic_drive_train {
HolonomicDriveTrainSubsystem::HolonomicDriveTrainSubsystem(
    std::unique_ptr<IHolonomicDrive> &drive_train)
    : m_drive_train(std::move(drive_train)),
      ASubsystem(ESubsystem::HOLONOMIC_DRIVE_TRAIN) {}

void HolonomicDriveTrainSubsystem::init() { m_drive_train->init(); }

void HolonomicDriveTrainSubsystem::run() { m_drive_train->run(); }

void HolonomicDriveTrainSubsystem::command(const commands::Command &cmd) {
  std::visit([this](auto &&cmd) { this->handleCommand(cmd); }, cmd);
}

void *HolonomicDriveTrainSubsystem::state(ESubsystemState state_name) {
  void *out = nullptr;
  switch (state_name) {
  case ESubsystemState::HOLONOMIC_DRIVE_TRAIN_GET_WHEEL_SPEED: {
    double *wheel_speeds = new double[4];
    for (int i = 0; i < 4; ++i) {
      wheel_speeds[i] = m_drive_train->getWheelSpeed(i);
    }
    out = wheel_speeds;
    break;
  }
  }
  return out;
}

void HolonomicDriveTrainSubsystem::handleCommand(
    const commands::holonomic_drive_train::SetMotionVectorCommand &cmd) {
  if (cmd.m_is_normal)
    m_drive_train->setNormalizedMotionVector(
        {cmd.m_x_velocity, cmd.m_y_velocity, cmd.m_angular_velocity});
  m_drive_train->setMotionVector(
      {cmd.m_x_velocity, cmd.m_y_velocity, cmd.m_angular_velocity});
}

void HolonomicDriveTrainSubsystem::handleCommand(
    const commands::holonomic_drive_train::SetLinearVelocityCommand &cmd) {
  if (cmd.m_is_normal)
    m_drive_train->setNormalizedLinearVelocity(cmd.m_x_velocity,
                                               cmd.m_y_velocity);
  m_drive_train->setLinearVelocity(cmd.m_x_velocity, cmd.m_y_velocity);
}

void HolonomicDriveTrainSubsystem::handleCommand(
    const commands::holonomic_drive_train::SetAngularVelocityCommand &cmd) {
  if (cmd.m_is_normal)
    m_drive_train->setNormalizedAngularVelocity(cmd.m_angular_velocity);
  m_drive_train->setAngularVelocity(cmd.m_angular_velocity);
}
} // namespace driftless::robot::subsystems::holonomic_drive_train