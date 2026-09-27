#ifndef __I_CLAW_HPP__
#define __I_CLAW_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief Namespace containing the claw subsystem
/// @author Matthew Backman
namespace claw {

/// @brief Generic claw driver class
/// @author Matthew Backman
class IClaw {
 public:
  /// @brief Destroy the IClaw object
  virtual ~IClaw() = default;

  /// @brief Initialize the IClaw object
  virtual void init() = 0;

  /// @brief Run the IClaw object
  virtual void run() = 0;

  /// @brief Set the rotation of the elbow joint of the claw
  /// @param target __float__ The target rotation of the elbow joint
  virtual void setElbowRotation(float target) = 0;

  /// @brief Set the rotation of the wrist joint of the claw
  /// @param target __float The target rotation of the wrist joint
  virtual void setWristRotation(float target) = 0;

  /// @brief Flips the claw upside down
  virtual void flipClaw() = 0;

  /// @brief Get the rotation of the elbow joint of the claw
  /// @return  __float__ The rotation of the elbow joint
  virtual float getElbowRotation() = 0;

  /// @brief Get the rotation of the wrist joint of the claw
  /// @return  __float__ The rotation of the wrist joint
  virtual float getWristRotation() = 0;

  /// @brief Determines if the claw is flipped
  /// @return __bool__ Returns true if the claw is flipped, false if the claw is
  /// not flipped
  virtual bool isClawFlipped() = 0;
};
}  // namespace claw
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif