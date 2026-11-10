#ifndef __ARM_SUBSYSTEM_HPP__
#define __ARM_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/arm/IArm.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief Namespace for the arm subsystem
/// @author Matthew Backman
namespace arm {

/// @brief Class for the arm subsystem of the robot
/// @author Matthew Backman
class ArmSubsystem : public ASubsystem {
 public:
  /// @brief Construct a new Arm Subsystem object
  /// @param arm_driver __std::unique_ptr<IArm>__ The driver class for the arm
  ArmSubsystem(std::unique_ptr<IArm> arm_driver);
  
  /// @brief Initialize the arm subsystem
  void init() override;

  /// @brief Run the arm subsystem
  void run() override;

  /// @brief Send a command to the arm subsystem
  /// @param cmd __const commands::Command&__ The command to send
  void command(const commands::Command& cmd) override;

  /// @brief Retrieve a state of the arm subsystem
  /// @param state __ESubsystemState__ The state to retrieve
  /// @return __void*__ The value of the desired state
  void* state(ESubsystemState state) override;

 private:
  std::unique_ptr<IArm> m_arm{};

  void handleCommand(const commands::arm::ArmGoLoadCommand cmd);
  void handleCommand(const commands::arm::ArmGoScanCommand cmd);
  void handleCommand(const commands::arm::ArmGoScoreLowCommand cmd);
  void handleCommand(const commands::arm::ArmGoScoreHighCommand cmd);
  void handleCommand(const commands::arm::ArmResetCommand cmd);
  void handleCommand(const auto cmd);
};
}  // namespace arm
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif