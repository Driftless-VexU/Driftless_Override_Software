#include "driftless/robot/subsystems/aligner/Aligner.hpp"

#include "driftless/robot/subsystems/aligner/AlignerBuilder.hpp"
namespace driftless::robot::subsystems::aligner {
void Aligner::init() {}

void Aligner::run() {}

void Aligner::setState(bool deployed) {
  if (deployed) {
    m_piston->extend();
  } else {
    m_piston->retract();
  }
}

void Aligner::toggleState() { m_piston->toggleState(); }

bool Aligner::isDeployed() { return m_piston->isExtended(); }

Aligner::Aligner(AlignerBuilder&& builder)
    : m_piston(std::move(builder.m_piston)) {}
}  // namespace driftless::robot::subsystems::aligner