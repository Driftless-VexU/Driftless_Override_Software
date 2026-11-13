#ifndef __Claw_HPP__
#define __Claw_HPP__

#include <memory>

#include "driftless/control/PID.hpp"
#include "driftless/hal/MotorGroup.hpp"
#include "driftless/io/IPiston.hpp"
#include "driftless/robot/subsystems/claw/IClaw.hpp"
#include "driftless/rtos/IDelayer.hpp"
#include "driftless/rtos/IMutex.hpp"
#include "driftless/rtos/ITask.hpp"

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
  /// @param is_up __bool__ Whether the claw should be facing up or down. True
  /// for up, false for down
  void setUpDown(bool is_up) override;

  /// @brief Toggle the orientation of the claw pivot between up and down
  void toggleUpDown() override;

  /// @brief Set the open/close state of the claw
  /// @param isOpen __bool__ Whether the claw should be open or closed. True for
  /// open, false for close
  void setOpenClosed(bool is_open) override;

  /// @brief Toggle the open/close state of the claw
  void toggleOpenClosed() override;

  /// @brief Determine if the claw is right-side-up or upside-down
  /// @return __bool__ True for right-side-up, false for upside-down
  bool isClawUp() override;

 private:
  static constexpr uint8_t TASK_DELAY{20};

  /// @brief Run the task update function of the provided claw in an endless
  /// loop
  /// @param params __void*__ Pointer to the claw to update
  static void taskLoop(void* params);

  hal::MotorGroup m_pivot_motor{};

  control::PID m_pivot_pid{};

  std::unique_ptr<io::IPiston> m_piston{};

  std::unique_ptr<rtos::ITask> m_task{};

  std::unique_ptr<rtos::IMutex> m_mutex{};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  float m_pivot_flipped_position{};

  float m_target_pivot_position{};

  float m_pivot_position_tolerance{};

  bool m_is_up{};

  /// @brief Construct a new Claw object
  /// @param builder __ClawBuilder&&__ The builder creating this
  /// Claw object. Consumed on use.
  Claw(ClawBuilder&& builder);

  /// @brief Update the state of the claw. Called periodically through taskLoop
  void taskUpdate();
};
}  // namespace claw
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif