#ifndef __ORANGE_SAFE_MATCH_AUTON_HPP__
#define __ORANGE_SAFE_MATCH_AUTON_HPP__

#include "driftless/auton/AAuton.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for autonomous routines
/// @author Matthew Backman
namespace auton {

class OrangeSafeMatchAuton : public AAuton {
 public:
  /// @brief Construct a new Orange Safe Match Auton object
  OrangeSafeMatchAuton();

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

  control::Point m_first_pin_grab{130.75, 88.75, M_PI / 5.0};

  control::Point m_first_pin_score_lineup{124.0, 70.0, M_PI / 2.0};

  control::Point m_line_pin_lineup{92.0, 70.0, -M_PI / 2.0};

  control::Point m_line_pin{95.0, 82.5, -M_PI / 2.0};

  control::Point m_right_alliance_goal{118.0, 55.0, -M_PI / 2.0};

  control::Point m_mid_pin_lineup{118.0, 64.0, -M_PI / 2.0};

  control::Point m_mid_pin{100.5, 53.75, -3.0 * M_PI / 4.0};

  control::Point m_mid_pin_score_lineup{118.5, 70.0, -M_PI / 2.0};

  control::Point m_front_wall_pin_lineup{90.0, 46.0, -2 * M_PI / 3.0};

  control::Point m_front_wall_pin{50.5, 10.0, -2.0 * M_PI / 3.0};

  control::Point m_left_neutral_goal{54.0, 18.0, 3.0 * M_PI / 4.0};

  control::Point m_middle_goal_lineup{70.0, 30.0, M_PI / 2.0};

  control::Point m_middle_goal{71.0, 56.0, M_PI / 2.0};
};
}  // namespace auton
}  // namespace driftless

#endif