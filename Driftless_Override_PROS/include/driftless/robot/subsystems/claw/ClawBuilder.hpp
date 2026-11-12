
#ifndef __CLAW_BUILDER_HPP__
#define __CLAW_BUILDER_HPP__

#include "driftless/robot/subsystems/claw/Claw.hpp"

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
/// @author Jack LaFreniere
namespace claw {

/// @brief Builder for Claw objects
/// @author Jack LaFreniere
/// @author Matthew Backman
class ClawBuilder {
  friend class Claw;

 public:
  /// @brief Add a piston to the builder
  /// @param piston __std::unique_ptr<io::IPiston>__ The piston to use
  /// @return __ClawBuilder&&__ Consumable reference to the active builder
  ClawBuilder&& withPiston(std::unique_ptr<io::IPiston> piston) &&;

  /// @brief Adds a motor to the builder
  /// @param motor __std::unique_ptr<io::IMotor>__ The motor being added
  /// @return __ClawBuilder&&__ Consumable reference to the current builder
  ClawBuilder&& withMotor(std::unique_ptr<io::IMotor> motor) &&;

  /// @brief Add a PID controller for the pivot motor to the builder
  /// @param pivot_pid __control::PID__ The PID controller to use for the pivot
  /// @return __ClawBuilder&&__ Consumable reference to the current builder
  ClawBuilder&& withPivotPID(control::PID pivot_pid) &&;

  /// @brief Add a task to the builder
  /// @param task __std::unique_ptr<rtos::ITask>__ The task to add
  /// @return __ClawBuilder&&__ Consumable reference to the current builder
  ClawBuilder&& withTask(std::unique_ptr<rtos::ITask> task) &&;

  /// @brief Add a mutex to the builder
  /// @param mutex __std::unique_ptr<rtos::IMutex>__ The mutex to add
  /// @return __ClawBuilder&&__ Consumable reference to the current builder
  ClawBuilder&& withMutex(std::unique_ptr<rtos::IMutex> mutex) &&;

  /// @brief Add a delayer to the builder
  /// @param delayer __std::unique_ptr<rtos::IDelayer>__ The delayer to add
  /// @return __ClawBuilder&&__ Consumable reference to the current builder
  ClawBuilder&& withDelayer(std::unique_ptr<rtos::IDelayer> delayer) &&;

  /// @brief Adds the number of motor rotations required to flip the pivot 180
  /// degrees to the builder
  /// @param pivot_flipped_rotations __float__ The motor rotations required to
  /// flip the pivot 180 degrees
  /// @return __ClawBuilder&&__ Consumable reference to the current builder
  ClawBuilder&& withPivotFlippedPosition(float pivot_flipped_position) &&;

  /// @brief Adds the tolerance of the position of the pivot
  /// @param pivot_position_tolerance __float__ The tolerance for the motor
  /// position of the pivot
  /// @return __ClawBuilder&&__ Consumable reference to the current builder
  ClawBuilder&& withPivotPositionTolerance(float pivot_position_tolerance) &&;

  /// @brief Builds a new Claw object
  /// @return __Claw__ The newly constructed Claw
  /// object
  [[nodiscard]]
  Claw build() &&;

  /// @brief Builds a new Claw object wrapped in a unique pointer
  /// @return __std::unique_ptr<Claw>__ Unique pointer to the
  /// newly constructed Claw object
  [[nodiscard]]
  std::unique_ptr<Claw> buildUnique() &&;

 private:
  hal::MotorGroup m_pivot_motor{};

  control::PID m_pivot_pid{};

  std::unique_ptr<io::IPiston> m_piston{};

  std::unique_ptr<rtos::ITask> m_task{};

  std::unique_ptr<rtos::IMutex> m_mutex{};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  float m_pivot_flipped_position{};

  float m_pivot_position_tolerance{};
};
}  // namespace claw
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif