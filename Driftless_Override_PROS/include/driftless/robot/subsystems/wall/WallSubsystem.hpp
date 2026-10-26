#ifndef __WALL_SUBSYSTEM_HPP__
#define __WALL_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/wall/IWall.hpp"

/// @brief The namespace for driftless library code
/// @author Samuel Throckmorton
namespace driftless {

/// @brief The namespace for robot code
/// @author Samuel Throckmorton
namespace robot {

/// @brief The namespace for subsystems code
/// @author Samuel Throckmorton
namespace subsystems {

/// @brief The namespace for the wall subsystem
/// @author Samuel Throckmorton
namespace wall {

/// @brief Class to adapt a wall driver to the subsystem structure
/// @author Samuel Throckmorton
class WallSubsystem : public ASubsystem {
 public:
  /// @brief Construct a new Wall Subsystem object
  /// @param wall_driver __std::unique_ptr<IWall>__ The wall driver class to
  /// use by the subsystem
  WallSubsystem(std::unique_ptr<IWall> wall_driver);

  /// @brief Initializes the wall subsystem 
  void init() override;

  /// @brief Runs the wall subsystem
  void run() override;

  /// @brief Send a command to the wall subsystem  
  /// @param cmd __const commands::Command&__ The command to send
  void command(const commands::Command& cmd) override;

  /// @brief Retrieves a state from the wall subsystem  
  /// @param state __ESubsystemState__ The state to retrieve from the wall subsystem
  /// @return __void*__ The value of the desired state
  void* state(ESubsystemState state) override;

 private:
  std::unique_ptr<IWall> m_wall_driver{};

  /// @brief Handles the set wall state command
  /// @param cmd __const commands::wall::SetWallStateCommand__ The command to
  /// handle
  void handleCommand(const commands::wall::SetWallStateCommand cmd);

  /// @brief Handles the toggle wall state command
  /// @param cmd __const commands::wall::ToggleWallStateCommand__ The command
  /// to handle
  void handleCommand(const commands::wall::ToggleWallStateCommand cmd);

  /// @brief Handles any commands that are not explicitly handled by the wall
  /// subsystem
  /// @param cmd __const auto__ The command to handle
  void handleCommand(const auto cmd);
};
}  // namespace wall
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif