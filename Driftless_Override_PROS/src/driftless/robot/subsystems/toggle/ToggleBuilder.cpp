#include "driftless/robot/subsystems/toggle/ToggleBuilder.hpp"

namespace driftless::robot::subsystems::toggle {
ToggleBuilder&& ToggleBuilder::withMotor(std::unique_ptr<io::IMotor> motor) && {
  m_motor.addMotor(std::move(motor));
  return std::move(*this);
}
std::unique_ptr<Toggle> ToggleBuilder::buildUnique() && {
  return std::unique_ptr<Toggle>{new Toggle{std::move(*this)}};
}
Toggle ToggleBuilder::build() && { return Toggle(std::move(*this)); }
}  // namespace driftless::robot::subsystems::toggle