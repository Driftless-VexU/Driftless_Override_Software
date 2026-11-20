#ifndef __BLUE_MATCH_AUTON_HPP__
#define __BLUE_MATCH_AUTON_HPP__

#include "driftless/auton/AAuton.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for autonomous routines
/// @author Matthew Backman
namespace auton {

class BlueMatchAuton : public AAuton {
 public:
  /// @brief Construct a new Orange Match Auton object
  BlueMatchAuton();

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

  control::Point m_starting_pos{64.0, 11.0, M_PI/2};

  control::Point m_first_goal{54.0, 20.0, 3*(M_PI)/4};

  control::Point m_second_goal_prepare_pos{68,50,0};
  
  control::Point m_middle_intake_pin{60.0,50,0};

  control::Point m_short_first_goal{87,32,15*(M_PI)/4};

  control::Point m_get_readey_to_approach_match_load{87,50,-M_PI/2};

  control::Point m_move_thing_approach_spot{105,32,-M_PI/2};

  control::Point m_stand_match_loader{127,20,-M_PI/2};

  control::Point m_match_loader{126,17,-M_PI/2};
};
}  // namespace auton
}  // namespace driftless
#endif