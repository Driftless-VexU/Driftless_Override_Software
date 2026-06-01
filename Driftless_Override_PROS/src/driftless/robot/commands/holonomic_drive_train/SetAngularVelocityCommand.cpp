#include "driftless/robot/commands/holonomic_drive_train/SetAngularVelocityCommand.hpp"

namespace driftless::robot::commands::holonomic_drive_train {
SetAngularVelocityCommand::SetAngularVelocityCommand(double angular_velocity,
                                                     bool is_normal)
    : m_angular_velocity(angular_velocity), m_is_normal(is_normal) {}

bool SetAngularVelocityCommand::isNormal() const { return m_is_normal; }

double SetAngularVelocityCommand::getAngularVelocity() const {
  return m_angular_velocity;
}
}  // namespace driftless::robot::commands::holonomic_drive_train