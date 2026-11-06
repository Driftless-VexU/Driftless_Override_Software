#ifndef __I_ARM_HPP__
#define __I_ARM_HPP__

#include <cstdint>

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

  /// @brief List of states the arm can be in
  /// @author Matthew Backman
enum class EArmState : uint8_t {
  START,
  INITIALIZED,
  LOAD,
  SCAN,
  SCORE_LOW,
  SCORE_HIGH,
};

/// @brief Interface for a generic arm driver with an elbow and wrist joint
/// @author Matthew Backman
class IArm {
  public:
  /// @brief Virtual destructor
  virtual ~IArm() = default;

  /// @brief move the wrist to a hard-stop and reset position
  virtual void resetWrist() = 0;

  /// @brief Move the arm to the load position
  virtual void goToLoadPosition() = 0;

  /// @brief Move the arm to the scan position
  virtual void goToScanPosition() = 0;

  /// @brief Move the arm to the lower scoring position
  virtual void goToScoreLowPosition() = 0;

  /// @brief Move the arm to the higher scoring position
  virtual void goToScoreHighPosition() = 0;

  /// @brief Get the current position state of the arm
  /// @return __EArmState__ The current position state of the arm
  virtual EArmState getCurrentPositionState() = 0;
};
}  // namespace arm
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif