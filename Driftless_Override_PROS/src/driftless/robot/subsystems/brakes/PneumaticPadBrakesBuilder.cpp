#include "driftless/robot/subsystems/brakes/PneumaticPadBrakesBuilder.hpp"

namespace driftless::robot::subsystems::brakes {
PneumaticPadBrakesBuilder&& PneumaticPadBrakesBuilder::withPiston(std::unique_ptr<io::IPiston> piston)&& {
    m_pistons = std::move(piston);
    return std::move(*this);
}
std::unique_ptr<PneumaticPadBrakes> PneumaticPadBrakesBuilder::buildUnique() &&{
    return std::unique_ptr<PneumaticPadBrakes>{new PneumaticPadBrakes(std::move(*this))};
}
PneumaticPadBrakes PneumaticPadBrakesBuilder::build() &&{
    return PneumaticPadBrakes(std::move(*this));
}
}