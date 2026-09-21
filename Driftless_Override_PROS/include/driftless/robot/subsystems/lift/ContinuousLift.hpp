#ifndef __CONTINUOUS_LIFT_HPP__
#define __CONTINUOUS_LIFT_HPP__

#include "driftless/robot/subsystems/lift/ILift.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the lift subsystem code
/// @author Matthew Backman
namespace lift {

class ContinuousLift : public ILift {
public:
  /// @brief Initialize the lift
  /// @author Matthew Backman
  void init() override;

  /// @brief Run the lift
  /// @author Matthew Backman
  void run() override;

private:
};
} // namespace lift
} // namespace subsystems
} // namespace robot
} // namespace driftless

#endif