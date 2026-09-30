#ifndef __PNEUMATIC_PAD_BRAKES_HPP__
#define __PNEUMATIC_PAD_BRAKES_HPP__

#include <memory>

#include "driftless/io/IPiston.hpp"
#include "driftless/robot/subsystems/brakes/IBrake.hpp"

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

// builder declaration, see PneumaticPadBrakesBuilder.hpp for definition
class PneumaticPadBrakesBuilder;

/// @brief Brakes implementation using pneumatics to control state
/// @author Matthew Backman
class PneumaticPadBrakes : public IBrake {
  friend class PneumaticPadBrakesBuilder;

 public:
  /// @brief Initialize the PneumaticPadBrakes object
  void init() override;

  /// @brief Run the PneumaticPadBrakes object
  void run() override;

  /// @brief Set the state of the brakes
  /// @param deployed __bool__ Whether the brakes are deployed or not. True for
  /// deployed, false for retracted.
  void setState(bool deployed) override;

  /// @brief Toggle the state of the brakes. I.e. retract them if deployed,
  /// deploy them if retracted.
  void toggleState() override;

  /// @brief Determine if the brakes are deployed
  /// @return __bool__ Whether the brakes are deployed or not. True for
  /// deployed, false for retracted.
  bool isDeployed() override;

 private:
  std::unique_ptr<io::IPiston> m_pistons{};

  /// @brief Construct a new Pneumatic Pad Brakes object
  /// @param builder __PneumaticPadBrakesBuilder&&__ The builder creating this
  /// PneumaticPadBrakes object. Consumed on use.
  PneumaticPadBrakes(PneumaticPadBrakesBuilder&& builder);
};
}  // namespace brakes
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif