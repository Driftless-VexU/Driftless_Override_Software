#include "driftless/auton/BlueMatchAuton.hpp"

#include "pros/screen.hpp"
namespace driftless::auton {
BlueMatchAuton::BlueMatchAuton() : AAuton("BLUE_MATCH") {}

void BlueMatchAuton::init(
    std::shared_ptr<driftless::robot::Robot>& robot,
    std::shared_ptr<driftless::control::ControlSystem>& control_system) {
  m_robot = robot;
  m_control_system = control_system;
}

void BlueMatchAuton::run(
    std::shared_ptr<driftless::robot::Robot>& robot,
    std::shared_ptr<driftless::control::ControlSystem>& control_system,
    std::shared_ptr<driftless::alliance::IAlliance>& alliance,
    std::shared_ptr<rtos::IClock>& clock,
    std::unique_ptr<rtos::IDelayer> delayer) {
  m_robot = robot;
  m_control_system = control_system;
  m_alliance = alliance;
  m_clock = clock;
  m_delayer = delayer->clone();

  uint32_t start_time{getTime()};
  setOdomPosition(m_starting_pos.getX(), m_starting_pos.getY(),
                  m_starting_pos.getTheta());

  // Spin toggle
  toggleSpin();
  closeClaw();
  delay(400);
  toggleStop();
}
}  // namespace driftless::auton