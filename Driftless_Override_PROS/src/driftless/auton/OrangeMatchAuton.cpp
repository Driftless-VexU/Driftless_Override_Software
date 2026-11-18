#include "driftless/auton/OrangeMatchAuton.hpp"

#include "pros/screen.hpp"
namespace driftless::auton {
OrangeMatchAuton::OrangeMatchAuton() : AAuton("ORANGE_MATCH") {}

void OrangeMatchAuton::init(
    std::shared_ptr<driftless::robot::Robot>& robot,
    std::shared_ptr<driftless::control::ControlSystem>& control_system) {
  m_robot = robot;
  m_control_system = control_system;
}

void OrangeMatchAuton::run(
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

  // spin right side toggle to our color, grab preload from loader
  toggleSpin();
  closeClaw();
  delay(400);
  toggleStop();

  // raise lift to prepare to score, callibrate arm
  setLiftHeight(10.0);
  deployAligner();
  armReset();

  // go to goal and score preload
  goToPose(m_right_neutral_goal, MAX_LINEAR_VELOCITY / 3.0,
           MAX_ANGULAR_VELOCITY, 20.0);
  waitForGoToPose(m_right_neutral_goal, 1.0, 2000);
  stopMotion();
  armGoScoreLow();
  setLiftHeight(1.0);
  delay(350);
  openClaw();
  retractAligner();
  delay(300);

  // go to first (of many) pins
  setLiftHeight(6.0);
  delay(500);
  turnToHeading(m_first_pin_grab.getTheta(), MAX_ANGULAR_VELOCITY / 3.0);
  waitForTurnToAngle(m_first_pin_grab.getTheta(), M_PI / 8.0, 2000);
  stopMotion();
  setLiftHeight(0.75);
  delay(250);
  goToPose(m_first_pin_grab, MAX_LINEAR_VELOCITY / 6.0, MAX_ANGULAR_VELOCITY,
           10.0);
  waitForGoToPose(m_first_pin_grab, 1.0, 1500);
  closeClaw();
  delay(200);

  // score first pin and cup
  setLiftHeight(14.0);
  goToPoint(m_first_pin_score_lineup, MAX_LINEAR_VELOCITY / 2.0);
  waitForGoToPoint(m_first_pin_score_lineup, 3.0, 1000);
  turnToHeading(m_right_neutral_goal.getTheta(), MAX_ANGULAR_VELOCITY);
  waitForTurnToAngle(m_right_neutral_goal.getTheta(), M_PI / 8.0, 1000);
  deployAligner();
  delay(200);
  goToPose(m_right_neutral_goal, MAX_LINEAR_VELOCITY / 3.0,
           MAX_ANGULAR_VELOCITY / 4.0, 20.0);
  waitForGoToPose(m_right_neutral_goal, 1.0, 2000);
  stopMotion();
  setLiftHeight(6.0);
  delay(250);
  openClaw();
  retractAligner();
  delay(250);

  // grab pin from the line for second goal
  setLiftHeight(16.0);
  delay(250);
  goToPose(m_line_pin_lineup, MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY / 2.0,
           40.0);
  delay(200);
  armGoLoad();
  setLiftHeight(4.0);
  waitForGoToPose(m_line_pin_lineup, 4.0, 2000);
  lowerIntake();
  runIntakeIn();
  goToPoint(m_line_pin, MAX_LINEAR_VELOCITY / 3.0);
  waitForGoToPoint(m_line_pin, 1.0, 2000);
  stopMotion();
  delay(500);
  raiseIntake();
  stopIntake();
  delay(100);

  // go to alliance goal to score line pin
  deployAligner();
  setLiftHeight(1.5);
  goToPose(m_right_alliance_goal, MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY,
           40.0);
  waitForGoToPose(m_right_alliance_goal, 8.0, 2000);
  setMotionLinearVelocity(MAX_LINEAR_VELOCITY / 4.0);
  closeClaw();
  delay(200);
  setLiftHeight(6.0);
  waitForGoToPose(m_right_alliance_goal, 8.0, 2000);
  delay(500);
  armGoScoreLow();
  delay(250);
  setLiftHeight(0.0);
  delay(250);
  openClaw();
  delay(750);
  setLiftHeight(4.0);
  retractAligner();

  // grab next cup and pin
  goToPose(m_mid_pin, MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(m_mid_pin, 10.0, 2000);
  setLiftHeight(0.75);
  setMotionLinearVelocity(MAX_LINEAR_VELOCITY / 6.0);
  waitForGoToPose(m_mid_pin, 1.0, 2000);
  closeClaw();
  delay(300);
  setLiftHeight(6.0);

  // leave at end
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 8, "Runtime: %7.2f",
                      (getTime() - start_time) / 1000.0);
}
}  // namespace driftless::auton