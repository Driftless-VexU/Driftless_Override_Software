#ifndef __ORANGE_MATCH_AUTON_HPP__
#define __ORANGE_MATCH_AUTON_HPP__

#include "driftless/auton/AAuton.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for autonomous routines
/// @author Matthew Backman
namespace auton {

class OrangeMatchAuton : public AAuton {
 public:
  /// @brief Construct a new Orange Match Auton object
  OrangeMatchAuton();

  /// @brief Initializes the auton
  /// @param robot __std::shared_ptr<robot::Robot>&__ The robot being controlled
  /// @param control_system __std::shared_ptr<control::ControlSystem>&__ The
  /// control system used
  /// @param process_system __std::shared_ptr<processes::ProcessSystem>&__ The
  /// process system used
  void init(std::shared_ptr<driftless::robot::Robot>& robot,
            std::shared_ptr<driftless::control::ControlSystem>& control_system)
      override;

  /// @brief Runs the auton
  /// @param robot __std::shared_ptr<robot::Robot>&__ The robot being controlled
  /// @param control_system __std::shared_ptr<control::ControlSystem>&__ The
  /// control system used
  /// @param process_system __std::shared_ptr<processes::ProcessSystem>&__ The
  /// process system used
  /// @param alliance __std::shared_ptr<alliance::IAlliance>&__ The current
  /// alliance
  /// @param clock __std::shared_ptr<rtos::IClock>&__ The system clock
  /// @param delayer __std::unique_ptr<rtos::IDelayer>__ The delayer used
  void run(std::shared_ptr<driftless::robot::Robot>& robot,
           std::shared_ptr<driftless::control::ControlSystem>& control_system,
           std::shared_ptr<driftless::alliance::IAlliance>& alliance,
           std::shared_ptr<rtos::IClock>& clock,
           std::unique_ptr<rtos::IDelayer> delayer) override;

 private:
  static constexpr float MAX_LINEAR_VELOCITY{80.0};

  static constexpr double MAX_ANGULAR_VELOCITY{M_PI * 4.0};

  control::Point m_starting_pos{132.75, 77.0, M_PI};

  control::Point m_right_neutral_goal{124.0, 86.0, M_PI * 3.0 / 4.0};

  control::Point m_first_pin_grab{132.5, 88.5, M_PI / 5.0};

  control::Point m_first_pin_score_lineup{130.0, 77.0, M_PI / 2.0};

  control::Point m_line_pin_lineup{94.0, 74.0, -M_PI / 2.0};

  control::Point m_line_pin{96.0, 83.0, -M_PI / 2.0};

  control::Point m_right_alliance_goal{118.5, 57.0, -M_PI / 2.0};

  control::Point m_mid_pin{102.0, 59.0, -3.0 * M_PI / 4.0};
};
}  // namespace auton
}  // namespace driftless

#endif