#ifndef __CONTROL_COMMANDS_HPP__
#define __CONTROL_COMMANDS_HPP__

#include <variant>

/* ADD COMMAND HEADERS HERE */
#include "driftless/control/commands/SetAngularVelocityCommand.hpp"
#include "driftless/control/commands/SetLinearVelocityCommand.hpp"
#include "driftless/control/commands/motion/DriveStraightCommand.hpp"
#include "driftless/control/commands/motion/GoToPointCommand.hpp"
#include "driftless/control/commands/motion/GoToPoseCommand.hpp"
#include "driftless/control/commands/motion/TurnToAngleCommand.hpp"
#include "driftless/control/commands/path/FollowPathCommand.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief The namespace for commands which can be sent to the control system
/// @author Matthew Backman
namespace commands {

using Command = std::variant<
    /* INSERT COMMAND TYPES HERE */
    SetLinearVelocityCommand, SetAngularVelocityCommand,
    motion::DriveStraightCommand, motion::GoToPointCommand,
    motion::GoToPoseCommand, motion::TurnToAngleCommand,
    path::FollowPathCommand>;

}  // namespace commands
}  // namespace control
}  // namespace driftless

#endif