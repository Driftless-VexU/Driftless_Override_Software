#include "driftless/robot/subsystems/aligner/AlignerBuilder.hpp"

namespace driftless::robot::subsystems::aligner {
AlignerBuilder&& AlignerBuilder::withPiston(
    std::unique_ptr<io::IPiston> piston) && {
  m_piston = std::move(piston);
  return std::move(*this);
}

std::unique_ptr<Aligner> AlignerBuilder::buildUnique() && {
  return std::unique_ptr<Aligner>{new Aligner(std::move(*this))};
}

Aligner AlignerBuilder::build() && { return Aligner(std::move(*this)); }
}  // namespace driftless::robot::subsystems::aligner