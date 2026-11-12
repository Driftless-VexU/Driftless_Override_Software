#ifndef __SET_CLAW_OPEN_CLOSE_COMMAND__
#define __SET_CLAW_OPEN_CLOSE_COMMAND__

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

/// @brief Command to set the open/close state of the claw. True for open, false
/// for closed.
/// @author Jack LaFreniere
struct SetClawOpenCloseCommand {
  bool m_isOpen{};
};
}  // namespace claw
}  // namespace commands
}  // namespace robot
}  // namespace driftless

#endif