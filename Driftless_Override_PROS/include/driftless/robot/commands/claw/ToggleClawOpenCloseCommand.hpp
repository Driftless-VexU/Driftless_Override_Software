#ifndef __TOGGLE_CLAW_OPEN_CLOSE_COMMAND__
#define __TOGGLE_CLAW_OPEN_CLOSE_COMMAND__

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
namespace claw {

/// @brief Command to toggle the claw between open and closed
/// @author Jack LaFreniere
struct ToggleClawOpenCloseCommand {};
}  // namespace claw
}  // namespace commands
}  // namespace robot
}  // namespace driftless

#endif