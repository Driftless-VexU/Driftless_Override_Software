#include "driftless/robot/commands/odometry/SetXPositionCommand.hpp"

namespace driftless::robot::commands::odometry {
SetXPositionCommand::SetXPositionCommand(double x_position)
    : m_x_position(x_position) {}

double SetXPositionCommand::getXPosition() const { return m_x_position; }
}