#ifndef __CONTINUOUS_LIFT_HPP__
#define __CONTINUOUS_LIFT_HPP__

#include <memory>

#include "driftless/hal/MotorGroup.hpp"
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

class ContinuousLiftBuilder;

class ContinuousLift : public ILift {
  friend class ContinuousLiftBuilder;

 public:
  /// @brief Initialize the lift
  /// @author Matthew Backman
  void init() override;

  /// @brief Run the lift
  /// @author Matthew Backman
  void run() override;

  /// @brief Set the Position of the lift
  /// @param position __float__ The position to target, in inches
  void setPosition(float position) override;

  /// @brief Get the Position of the lift
  /// @return  __float__ The position of the lift, in inches
  float getPosition() override;

 private:
  float m_position{};

  float m_target_position{};

  hal::MotorGroup m_motors{};

  ContinuousLift(ContinuousLiftBuilder&& builder);
};
}  // namespace lift
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif