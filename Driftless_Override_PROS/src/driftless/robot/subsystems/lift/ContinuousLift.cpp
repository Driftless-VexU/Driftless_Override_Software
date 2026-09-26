#include "driftless/robot/subsystems/lift/ContinuousLift.hpp"

namespace driftless::robot::subsystems::lift {
  void ContinuousLift::init() {}

  void ContinuousLift::run() {}

  void ContinuousLift::setPosition(float position) {
    m_target_position = position;
  }

  float ContinuousLift::getPosition() {
    return m_position;
  }

}