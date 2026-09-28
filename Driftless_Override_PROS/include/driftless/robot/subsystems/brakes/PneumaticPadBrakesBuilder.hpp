#ifndef __PNEUMATIC_PAD_BRAKES_BUILDER_HPP__
#define __PNEUMATIC_PAD_BRAKES_BUILDER_HPP__

#include "driftless/robot/subsystems/brakes/PneumaticPadBrakes.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the brakes subsystem
/// @author Matthew Backman
namespace brakes {

/// @brief Builder for PneumaticPadBrakes objects
/// @author Matthew Backman
class PneumaticPadBrakesBuilder {
  friend class PneumaticPadBrakes;

 public:
  /// @brief Add a piston to the builder
  /// @param piston __std::unique_ptr<io::IPiston>__ The piston to use
  /// @return __PneumaticPadBrakesBuilder&&__ Reference to the active builder
  PneumaticPadBrakesBuilder&& withPiston(std::unique_ptr<io::IPiston> piston) &&;

  /// @brief Builds a new PneumaticPadBrakes object
  /// @return __PneumaticPadBrakes__ The newly constructed PneumaticPadBrakes
  /// object
  [[nodiscard]]
  PneumaticPadBrakes build() &&;

  /// @brief Builds a new PneumaticPadBrakes object wrapped in a unique pointer
  /// @return __std::unique_ptr<PneumaticPadBrakes>__ Unique pointer to the
  /// newly constructed PneumaticPadBrakes object
  [[nodiscard]]
  std::unique_ptr<PneumaticPadBrakes> buildUnique() &&;

 private:
  std::unique_ptr<io::IPiston> m_pistons{};
};
}  // namespace brakes
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif