#ifndef __MOTION_CONTROL_HPP__
#define __MOTION_CONTROL_HPP__

#include <memory>

#include "driftless/control/AControl.hpp"
#include "driftless/control/commands/Commands.hpp"
#include "driftless/control/motion/EMotionType.hpp"
#include "driftless/control/motion/IDriveStraight.hpp"
#include "driftless/control/motion/IGoToPoint.hpp"
#include "driftless/control/motion/IGoToPose.hpp"
#include "driftless/control/motion/ITurn.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for basic motion control algorithms
/// @author Matthew Backman
namespace motion {

/// @brief Class to hold and control all motion algorithms
/// @note This class should be split up, and each motion type should be its own
/// control. Try to avoid making major changes here, instead use the opportunity
/// to split this class into more manageable pieces.
/// @author Matthew Backman
class MotionControl : public driftless::control::AControl {
 private:
  /// @brief The algorithm to drive straight
  std::unique_ptr<driftless::control::motion::IDriveStraight>
      m_drive_straight{};

  /// @brief The algorithm to go to a point
  std::unique_ptr<driftless::control::motion::IGoToPoint> m_go_to_point{};

  /// @brief The algorithm to go to a pose
  std::unique_ptr<driftless::control::motion::IGoToPose> m_go_to_pose{};

  /// @brief The algorithm to turn
  std::unique_ptr<driftless::control::motion::ITurn> m_turn{};

  /// @brief The current type of motion, defaults to __NONE__
  driftless::control::motion::EMotionType m_motion_type{EMotionType::NONE};

  /// @brief Changes the type of motion being used and pauses the previous
  /// motion type
  /// @param motion_type __EMotionType__ The new motion type to switch to
  void switchMotionType(EMotionType motion_type);

  /// @brief Handles the drive straight command
  /// @param cmd __DriveStraightCommand&__ The command to handle
  void handleCommand(const commands::motion::DriveStraightCommand& cmd);

  /// @brief Handles the go to point command
  /// @param cmd __GoToPointCommand&__ The command to handle
  void handleCommand(const commands::motion::GoToPointCommand& cmd);

  /// @brief Handles the go to pose command
  /// @param cmd __GoToPoseCommand&__ The command to handle
  void handleCommand(const commands::motion::GoToPoseCommand& cmd);

  /// @brief Handles the turn to angle command
  /// @param cmd __TurnToAngleCommand&__ The command to handle
  void handleCommand(const commands::motion::TurnToAngleCommand& cmd);

  /// @brief Handles the turn to point command
  /// @param cmd __TurnToPointCommand&__ The command to handle
  void handleCommand(const commands::motion::TurnToPointCommand& cmd);

  /// @brief Handles the set linear velocity command
  /// @param cmd __SetLinearVelocityCommand&__ The command to handle
  void handleCommand(const commands::SetLinearVelocityCommand& cmd) const;

  /// @brief Handles the set angular velocity command
  /// @param cmd __SetAngularVelocityCommand&__ The command to handle
  void handleCommand(const commands::SetAngularVelocityCommand& cmd) const;

  /// @brief Handles any command with no explicit handler
  /// @throws std::invalid_argument for all commands with no explicit handler
  /// @param cmd __auto&__ The command to handle
  void handleCommand(const auto& cmd) const;

 public:
  /// @brief Constructs a new Motion Control object
  /// @param drive_straight __std::unique_ptr<IDriveStraight>&__ The algorithm
  /// used to drive straight
  /// @param go_to_point __std::unique_ptr<IGoToPoint>&__ The algorithm used to
  /// go to a point
  /// @param go_to_pose __std::unique_ptr<IGoToPose>&__ The algorithm used to
  /// go to a pose
  /// @param turn __std::unique_ptr<ITurn>&__ The algorithm used to turn
  MotionControl(
      std::unique_ptr<driftless::control::motion::IDriveStraight>&
          drive_straight,
      std::unique_ptr<driftless::control::motion::IGoToPoint>& go_to_point,
      std::unique_ptr<driftless::control::motion::IGoToPose>& go_to_pose,
      std::unique_ptr<driftless::control::motion::ITurn>& turn);

  /// @brief Initializes the motion control
  void init() override;

  /// @brief Runs the motion control
  void run() override;

  /// @brief Pauses the motion control
  void pause() override;

  /// @brief Resumes the motion control
  void resume() override;

  /// @brief Sends a command to the motion control
  /// @param command_name __EControlCommand__ The name of the command to run
  /// @param args __va_list&__ Any arguments needed for the command
  void command(const commands::Command& command) override;

  /// @brief Gets a state of the motion control
  /// @param state_name __EControlState__ The name of the state desired
  /// @return __void*__ The desired state of the motion control
  void* state(EControlState state_name) override;
};
}  // namespace motion
}  // namespace control
}  // namespace driftless
#endif