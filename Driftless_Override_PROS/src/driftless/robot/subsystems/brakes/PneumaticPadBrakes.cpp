#include "driftless/robot/subsystems/brakes/PneumaticPadBrakes.hpp"

#include "driftless/robot/subsystems/brakes/PneumaticPadBrakesBuilder.hpp"
namespace driftless::robot::subsystems::brakes {
void PneumaticPadBrakes::init() {}

void PneumaticPadBrakes::run() {}

void PneumaticPadBrakes::setState(bool deployed) {
  if (deployed) {
    m_pistons->extend();
  } else {
    m_pistons->retract();
  }
}

void PneumaticPadBrakes::toggleState() { m_pistons->toggleState(); }

bool PneumaticPadBrakes::isDeployed() { return m_pistons->isExtended(); }
PneumaticPadBrakes::PneumaticPadBrakes(PneumaticPadBrakesBuilder&& builder)
    : m_pistons(std::move(builder.m_pistons)) {}
}  // namespace driftless::robot::subsystems::brakes