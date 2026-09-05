#ifndef __CONTROL_COMMANDS_HPP__
#define __CONTROL_COMMANDS_HPP__

#include <variant>

/* ADD COMMAND HEADERS HERE */

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
    >;

}  // namespace commands
}  // namespace control
}  // namespace driftless

#endif