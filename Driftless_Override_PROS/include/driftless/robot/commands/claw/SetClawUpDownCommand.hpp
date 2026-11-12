#ifndef __SET_CLAW_UP_DOWN_COMMAND__
#define __SET_CLAW_UP_DOWN_COMMAND__

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

/// @brief Command to set the up/down state of the claw. True for up, false for
/// down
/// @author Jack LaFreniere
struct SetClawUpDownCommand {
  bool m_isUp{};
};
}  // namespace claw
}  // namespace commands
}  // namespace robot
}  // namespace driftless

#endif