#include "driftless/robot/subsystems/wall/WallBuilder.hpp"

namespace driftless::robot::subsystems::wall {
WallBuilder&& WallBuilder::withWallPiston(
    std::unique_ptr<io::IPiston> piston) && {
  m_piston_wall = std::move(piston);
  return std::move(*this);
}
WallBuilder&& WallBuilder::withWheelPiston(
    std::unique_ptr<io::IPiston> piston) && {
  m_piston_wheel = std::move(piston);
  return std::move(*this);
}
WallBuilder&& WallBuilder::withWheelMotor(
    std::unique_ptr<io::IMotor> motor) && {
  m_wheel_motor.addMotor(std::move(motor));
  return std::move(*this);
}
std::unique_ptr<Wall> WallBuilder::buildUnique() && {
  return std::unique_ptr<Wall>{new Wall(std::move(*this))};
}
Wall WallBuilder::build() && { return Wall(std::move(*this)); }
}  // namespace driftless::robot::subsystems::wall