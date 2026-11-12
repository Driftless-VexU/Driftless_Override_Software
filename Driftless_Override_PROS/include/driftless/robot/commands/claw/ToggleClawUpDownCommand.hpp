#ifndef __TOGGLE_CLAW_UP_DOWN_COMMAND_HPP__
#define __TOGGLE_CLAW_UP_DOWN_COMMAND_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for commands which can be sent to the robot
/// @author Matthew Backman
namespace commands {

/// @brief The namespace for commands which can be sent to the claw subsystem
/// @author Jack LaFreniere
/// @author Matthew Backman
namespace claw {

/// @brief Toggles the claw between right-side up and upside-down
/// @author Matthew Backman
struct ToggleClawUpDownCommand {};
}  // namespace claw
}  // namespace commands
}  // namespace robot
}  // namespace driftless

#endif