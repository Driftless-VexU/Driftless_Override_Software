#include "driftless/robot/commands/holonomic_drive_train/SetLinearVelocityCommand.hpp"

namespace driftless::robot::commands::holonomic_drive_train {
SetLinearVelocityCommand::SetLinearVelocityCommand(double x_velocity,
                                                   double y_velocity,
                                                   bool is_normal)
    : m_x_velocity(x_velocity),
      m_y_velocity(y_velocity),
      m_is_normal(is_normal) {}

bool SetLinearVelocityCommand::isNormal() const { return m_is_normal; }

double SetLinearVelocityCommand::getXVelocity() const { return m_x_velocity; }

double SetLinearVelocityCommand::getYVelocity() const { return m_y_velocity; }
}  // namespace driftless::robot::commands::holonomic_drive_train