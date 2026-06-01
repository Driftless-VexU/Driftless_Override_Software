#include "driftless/robot/commands/odometry/SetPositionCommand.hpp"

namespace driftless::robot::commands::odometry {
SetPositionCommand::SetPositionCommand(double x_pos, double y_pos,
                                       double theta_pos)
    : m_x_pos(x_pos), m_y_pos(y_pos), m_theta_pos(theta_pos) {}

double SetPositionCommand::getXPos() const { return m_x_pos; }

double SetPositionCommand::getYPos() const { return m_y_pos; }

double SetPositionCommand::getThetaPos() const { return m_theta_pos; }
}  // namespace driftless::robot::commands::odometry