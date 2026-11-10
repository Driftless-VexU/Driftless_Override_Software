#ifndef __I_WALL_HPP__
#define __I_WALL_HPP__

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
/// @author Samuel Throckmorton & Jacob Fiedorowicz
namespace wall {

class IWall {
 public:
  /// @brief Destroy the IWall object
  virtual ~IWall() = default;

  /// @brief Initialize the IWall object
  virtual void init() = 0;

  /// @brief Run the IWall object
  virtual void run() = 0;

  /// @brief Set the state of the wall
  /// @param deployed __bool__ Whether the wall is deployed or not. True for
  /// deployed, false for retracted.
  virtual void setWallExtensionState(bool deployed) = 0;

  /// @brief Toggle the state of the wall. I.e. retract it if deployed,
  /// deploy it if retracted.
  virtual void toggleWallExtensionState() = 0;

  /// @brief Determine if the wall is deployed
  /// @return __bool__ Whether the wall is deployed or not. True for
  /// deployed, false for retracted.
  virtual bool isWallExtended() = 0;

  /// @brief Set the state of the motor position
  /// @param deployed __bool__ Whether the flex-wheel is deployed or not. True
  /// for deployed, false for retracted
  virtual void setWallWheelState(bool deployed) = 0;

  /// @brief Toggles the state of the wheel on the wall. I.e. retract if
  /// deployed, deploy if retracted
  virtual void toggleWallWheelState() = 0;

  /// @brief Sets the speed for the wheel motor
  /// @param voltage __float__ The voltage that the motor has to spin the wheel.
  virtual void setWallWheelMotorVoltage(float voltage) = 0;
};
}  // namespace wall
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif