#ifndef __SET_ALIGNER_STATE_COMMAND_HPP__
#define __SET_ALIGNER_STATE_COMMAND_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for commands which can be sent to the robot
/// @author Matthew Backman
namespace commands {

/// @brief The namespace for commands which can be sent to the brakes subsystem
/// @author Jacob Fiedorowicz
namespace aligner {

/// @brief Command to set the state of the brakes
/// @author Jacob Fiedorowicz
struct SetAlignerStateCommand {
  bool m_deployed{};
};
}  // namespace aligner
}  // namespace commands
}  // namespace robot
}  // namespace driftless

#endif
