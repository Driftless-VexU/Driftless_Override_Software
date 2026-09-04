#ifndef __DRIVETRAIN_SUBSYSTEM_HPP__
#define __DRIVETRAIN_SUBSYSTEM_HPP__

#include <functional>
#include <map>
#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/tank_drive_train/ITankDriveTrain.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the drivetrain subsystem code
/// @author Matthew Backman
namespace tank_drive_train {

/// @brief The drive train subsystem
/// @author Matthew Backman
class TankDriveTrainSubsystem : public ASubsystem {
 private:
  std::unique_ptr<ITankDriveTrain> m_drive_train{};

  /// @brief Handles the set velocity command
  /// @param cmd __SetVelocityCommand&__ The command to handle
  void handleCommand(const commands::tank_drive_train::SetVelocityCommand& cmd);

  /// @brief Handles the set voltage command
  /// @param cmd __SetVoltageCommand&__ The command to handle
  void handleCommand(const commands::tank_drive_train::SetVoltageCommand& cmd);

  /// @brief Handles any command with no explicit handler
  /// @throws std::invalid_argument for all commands with no explicit handler
  /// @param cmd __auto&__ The command to handle
  void handleCommand(const auto& cmd);

 public:
  /// @brief Constructs a new drive train subsystem
  /// @param drivetrain __std::unique_ptr<IDrivetrain>&__ The drive train
  /// controller used by the subsystem
  TankDriveTrainSubsystem(std::unique_ptr<ITankDriveTrain>& drivetrain);

  /// @brief Initializes the subsystem
  void init() override;

  /// @brief Runs the subsystem
  void run() override;

  /// @brief Sends a command to the subsystem
  /// @param cmd __commands::Command&__ The command to send
  void command(const commands::Command& cmd) override;

  /// @brief Gets a state of the subsystem
  /// @param state_name __ESubsystemState__ The desired state
  /// @return __void*__ A pointer to the state of the subsystem
  void* state(ESubsystemState state_name) override;
};
}  // namespace tank_drive_train
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif