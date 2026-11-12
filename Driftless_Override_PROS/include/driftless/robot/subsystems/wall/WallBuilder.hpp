#ifndef __WALL_BUILDER_HPP__
#define __WALL_BUILDER_HPP__

#include "driftless/robot/subsystems/wall/Wall.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Jacob Fiedorowicz
namespace subsystems {

/// @brief The namespace for the wall subsystem
/// @author Jacob Fiedorowicz
namespace wall {

/// @brief Builder for wall object
/// @author Jacob Fiedorowicz
class WallBuilder {
  friend class Wall;

 public:
  /// @brief Add a piston to the builder for the wall
  /// @param piston __std::unique_ptr<io::IPiston>__ the piston to use
  /// @return __WallBuilder&&__ Reference to the active builder
  WallBuilder&& withWallPiston(std::unique_ptr<io::IPiston> piston) &&;

  /// @brief Add a piston to the builder for the wheel
  /// @param piston __std:unique_ptr<io::IPiston>__ The piston to use
  /// @return __WallBuilder&&__ Reference to the active builder
  WallBuilder&& withWheelPiston(std::unique_ptr<io::IPiston> piston) &&;

  /// @brief Add a motor to the builder
  /// @param motor __std::unique_ptr<io::IMotor>__ the motor to use
  /// @return __WallBuilder&&__ Reference to the active builder
  WallBuilder&& withWheelMotor(std::unique_ptr<io::IMotor> motor) &&;

  /// @brief Builds a new wall object
  /// @return __Wall__ The newly constructed wall object
  [[nodiscard]]
  Wall build() &&;

  /// @brief Builds a new wall object wrapped in a unique pointer
  /// @return __std::unique_ptr<Wall>__ Unique pointer to the newly constructed
  /// wall object
  [[nodiscard]]
  std::unique_ptr<Wall> buildUnique() &&;

  private:
  std::unique_ptr<io::IPiston> m_piston_wall;
  std::unique_ptr<io::IPiston> m_piston_wheel;
  hal::MotorGroup m_wheel_motor;
};
}  // namespace wall
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif