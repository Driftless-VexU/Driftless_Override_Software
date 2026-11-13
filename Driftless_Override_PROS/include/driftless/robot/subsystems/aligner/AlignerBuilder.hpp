#ifndef __ALIGNER_BUILDER_HPP__
#define __ALIGNER_BUILDER_HPP__

#include "driftless/robot/subsystems/aligner/Aligner.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the aligner subsystem
/// @author Jacob Fiedorowicz
namespace aligner {

/// @brief Builder for Aligner objects
/// @author Jacob Fiedorowicz
class AlignerBuilder {
  friend class Aligner;

 public:
  /// @brief Add a piston to the builder
  /// @param piston __std::unique_ptr<io::IPiston>__ The piston to use
  /// @return __AlignerBuilder&&__ Reference to the active builder
  AlignerBuilder&& withPiston(std::unique_ptr<io::IPiston> piston) &&;

  /// @brief Builds a new Aligner object
  /// @return __Aligner__ The newly constructed aligner object
  [[nodiscard]]
  Aligner build() &&;

  /// @brief Builds a new Aligner object wrapped in a unique pointer
  /// @return __std::unique_ptr<Aligner>__ Unique pointer to the newly
  /// constructed Aligner object
  [[nodiscard]]
  std::unique_ptr<Aligner> buildUnique() &&;

 private:
  std::unique_ptr<io::IPiston> m_piston{};
};
}  // namespace aligner
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif