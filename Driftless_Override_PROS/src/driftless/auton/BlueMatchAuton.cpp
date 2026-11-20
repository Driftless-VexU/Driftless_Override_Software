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

  // Setup for move
  setLiftHeight(10.0f);
  deployAligner();
  armReset();

  // Go to pos
  goToPose(m_first_goal, MAX_LINEAR_VELOCITY / 3.0, MAX_ANGULAR_VELOCITY, 20.0);
  waitForGoToPose(m_first_goal, 2.0, 1500);
  stopMotion();

  // Lower arm
  armGoScoreLow();
  delay(250);
  setLiftHeight(1.0);
  delay(250);
  openClaw();
  delay(250);
  setLiftHeight(10.0);
  delay(250);

  // Reset
  goToPose(m_starting_pos, MAX_LINEAR_VELOCITY / 3.0, MAX_ANGULAR_VELOCITY,
           20.0);
  waitForGoToPose(m_first_goal, 2.0, 1500);
  stopMotion();
  retractAligner();

  goToPose(m_second_goal_prepare_pos, MAX_LINEAR_VELOCITY / 3.0,
           MAX_ANGULAR_VELOCITY, 20.0);
  waitForGoToPose(m_second_goal_prepare_pos, 2.0, 2500);

  armGoLoad();
  lowerIntake();
  runIntakeIn();
  goToPose(m_middle_intake_pin, MAX_LINEAR_VELOCITY / 3.0, MAX_ANGULAR_VELOCITY,
           20.0);
  waitForGoToPose(m_middle_intake_pin, 2.0, 1500);
  delay(500);
  raiseIntake();

  goToPose(m_short_first_goal, MAX_LINEAR_VELOCITY / 3.0, MAX_ANGULAR_VELOCITY,
           20.0);
  waitForGoToPose(m_short_first_goal, 2.0, 6000);
  deployAligner();
  delay(100);
  stopIntake();
  delay(100);
  setLiftHeight(1.75);
  delay(250);
  closeClaw();
  delay(250);
  setLiftHeight(2.5);
  delay(250);
  armGoScoreLow();
  setLiftHeight(0.0);
  delay(500);
  openClaw();
  delay(250);
  setLiftHeight(4.0);
  retractAligner();
  delay(250);

  goToPose(m_get_readey_to_approach_match_load, MAX_LINEAR_VELOCITY / 3.0,
           MAX_ANGULAR_VELOCITY, 20.0);
  waitForGoToPose(m_get_readey_to_approach_match_load, 2.0, 1000);
  delay(100);

  goToPose(m_move_thing_approach_spot, MAX_LINEAR_VELOCITY / 4.5,
           MAX_ANGULAR_VELOCITY, 20.0);
  waitForGoToPose(m_move_thing_approach_spot, 2.0, 2000);

  delay(250);

  goToPose(m_stand_match_loader, MAX_LINEAR_VELOCITY / 4.0,
           MAX_ANGULAR_VELOCITY, 20.0);
  waitForGoToPose(m_stand_match_loader, 2.0, 2500);
  setLiftHeight(0);
  delay(150);

  goToPose(m_match_loader, MAX_LINEAR_VELOCITY / 5.0, MAX_ANGULAR_VELOCITY,
           20.0);
  waitForGoToPose(m_match_loader, 2.0, 1500);

  delay(300);
  closeClaw();
}
}  // namespace driftless::auton