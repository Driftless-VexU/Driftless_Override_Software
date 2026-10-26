#ifndef __SET_WALL_STATE_COMMAND_HPP__
#define __SET_WALL_STATE_COMMAND_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for commands which can be sent to the robot
/// @author Matthew Backman
namespace commands {

/// @brief The namespace for commands which can be sent to the wall subsystem
/// @author Matthew Backman
namespace wall {

/// @brief Command to set the state of the wall
/// @author Matthew Backman
struct SetWallStateCommand {
  bool m_deployed{};
};
}  // namespace wall
}  // namespace commands
}  // namespace robot
}  // namespace driftless

#endif