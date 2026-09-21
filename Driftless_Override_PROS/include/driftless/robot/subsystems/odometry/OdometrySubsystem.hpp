#ifndef __ODOMETRY_SUBSYSTEM_HPP__
#define __ODOMETRY_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/odometry/InertialPositionTracker.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for odometry code
/// @author Matthew Backman
namespace odometry {

/// @brief Class representing the odometry subsystem of the robot
/// @author Matthew Backman
class OdometrySubsystem : public ASubsystem {
 private:
  // the position tracker being used
  std::unique_ptr<IPositionTracker> m_position_tracker{};

  /// @brief Handles the SetHeadingCommand command
  /// @param cmd __SetHeadingCommand&__ The command to handle
  void handleCommand(const commands::odometry::SetHeadingCommand& cmd);

  /// @brief Handles the SetPositionCommand command
  /// @param cmd __SetPositionCommand&__ The command to handle
  void handleCommand(const commands::odometry::SetPositionCommand& cmd);

  /// @brief Handles the SetXPositionCommand command
  /// @param cmd __SetXPositionCommand&__ The command to handle
  void handleCommand(const commands::odometry::SetXPositionCommand& cmd);

  /// @brief Handles the SetYPositionCommand command
  /// @param cmd __SetYPositionCommand&__ The command to handle
  void handleCommand(const commands::odometry::SetYPositionCommand& cmd);

  /// @brief Handles any command with no explicit handler
  /// @throws std::invalid_argument for all commands with no explicit handler
  /// @param cmd __auto&__ The command to handle
  void handleCommand(const auto& cmd);

 public:
  /// @brief Constructor
  /// @param position_tracker __std::unique_ptr<IPositionTracker>&__ The
  /// position tracker to use
  OdometrySubsystem(std::unique_ptr<IPositionTracker>& position_tracker);

  /// @brief Initializes the subsystem
  void init() override;

  /// @brief Runs the subsystem
  void run() override;

  /// @brief Sends a command to the subsystem
  /// @param cmd __commands::Command&__ The command to send
  void command(const commands::Command& cmd) override;

  /// @brief Gets a specified state of the subsystem
  /// @param state_name __ESubsystemState__ The state to get
  /// @return __void*__ The state
  void* state(ESubsystemState state_name) override;
};

}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif