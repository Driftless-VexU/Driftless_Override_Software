#ifndef __LIFT_SET_POSITION_COMMAND_HPP__
#define __LIFT_SET_POSITION_COMMAND_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for commands which can be sent to the robot
/// @author Matthew Backman
namespace commands {

namespace lift {

struct SetPositionCommand {
  const float m_position{};
};

}  // namespace lift
}  // namespace commands
}  // namespace robot
}  // namespace driftless

#endif