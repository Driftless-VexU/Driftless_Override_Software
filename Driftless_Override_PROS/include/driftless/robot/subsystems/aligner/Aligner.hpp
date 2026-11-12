#ifndef __ALIGNER_HPP__
#define __ALIGNER_HPP__

#include <memory>

#include "driftless/io/IPiston.hpp"
#include "driftless/robot/subsystems/aligner/IAligner.hpp"

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

// builder declaration, see AlignerBuilder.hpp for definition
class AlignerBuilder;

/// @brief Aligner implementation using pneumatics to control state
/// @author Jacob Fiedorowicz
class Aligner : public IAligner {
  friend class AlignerBuilder;

 public:
  ///@brief Initialize the aligner
  void init() override;

  ///@brief Run the aligner
  void run() override;

  /// @brief Set the state of the aligner
  /// @param deployed __bool__ Whether the aligner is deployed or not, false for
  /// retracted, true for deployed
  void setState(bool deployed) override;

  /// @brief Toggles the state of the aligner. I.e. retract them if deployed,
  /// deploy them if retracted
  void toggleState() override;

  ///@brief Determine if the aligner is deployed
  /// @return __bool__ Whether the aligner is deployed or not, true for
  /// deployed, false for retracted
  bool isDeployed() override;

 private:
  std::unique_ptr<io::IPiston> m_piston{};

  /// @brief Constructs a new Aligner object
  /// @param builder __AlignerBuilder&&__ The aligner creating this aligner
  /// object. Consumed on use.
  Aligner(AlignerBuilder&& builder);
};
}  // namespace aligner
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif
