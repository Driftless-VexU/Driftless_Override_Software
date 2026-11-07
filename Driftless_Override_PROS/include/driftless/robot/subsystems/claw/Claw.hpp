#ifndef __Claw_HPP__
#define __Claw_HPP__

#include <memory>

#include "driftless/robot/subsystems/claw/IClaw.hpp"
#include "driftless/robot/subsystems/claw/ClawSubsystem.hpp"
#include "driftless/hal/MotorGroup.hpp"
#include "driftless/io/IPiston.hpp"

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

// builder declaration, see ClawBuilder.hpp for definition
class ClawBuilder;

/// @brief Interface for claw subsystem
/// @author Jack LaFreniere
class Claw : public IClaw {
  friend class ClawBuilder;

 public:
  /// @brief Initializes the Claw object
  void init() override;

  /// @brief Runs the Claw object
  void run() override;

  /// @brief Set the up/down state of the claw
  /// @param isUp __bool__ Whether the claw should be facing up or down. Up for
  /// true, down for false
  void setUpDown(bool isUp) override;

  /// @brief Set the open/close state of the claw
  /// @param isOpen __bool__ Whether the claw should be open or closed. Open for
  /// true, closed for false
  void setOpenClosed(bool isOpen) override;

  /// @brief toggle the open/close state of the claw
  void toggleOpenClosed() override;

  /// @brief Determines if the state of the claw is up
  bool isClawUp() override;

  private:
  hal::MotorGroup m_pivot_motor{};
  std::unique_ptr<io::IPiston> m_piston{};

  float m_motor_rotations_to_pivot_radians{};
  float m_target_pivot_position{};

  bool m_isUp{};

  /// @brief Construct a new Claw object
  /// @param builder __ClawBuilder&&__ The builder creating this
  /// Claw object. Consumed on use.
  Claw(ClawBuilder&& builder);
};
}  // namespace claw
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif