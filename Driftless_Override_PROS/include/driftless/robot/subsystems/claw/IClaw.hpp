#ifndef __I_Claw_HPP__
#define __I_Claw_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the holonomic drive train subsystem code
/// @author Matthew Backman
namespace claw {

/// @brief Interface for claw subsystem
/// @author Jack LaFreniere
class IClaw {
 public:
  /// @brief Destroys the IClaw object
  virtual ~IClaw() = default;

  /// @brief Initializes the IClaw object
  virtual void init() = 0;

  /// @brief Runs the IClaw object
  virtual void run() = 0;

  /// @brief Set the up/down state of the claw
  /// @param isUp __bool__ Whether the claw should be facing up or down. Up for
  /// true, down for false
  virtual void setUpDown(bool isUp) = 0;

  /// @brief Toggle the claw pivot between up and down
  virtual void toggleUpDown() = 0;

  /// @brief Set the open/close state of the claw
  /// @param isOpen __bool__ Whether the claw should be open or closed. Open for
  /// true, closed for false
  virtual void setOpenClosed(bool isOpen) = 0;

  /// @brief toggle the open/close state of the claw
  virtual void toggleOpenClosed() = 0;

  /// @brief Determines if the state of the claw is up
  virtual bool isClawUp() = 0;
};
}  // namespace claw
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif