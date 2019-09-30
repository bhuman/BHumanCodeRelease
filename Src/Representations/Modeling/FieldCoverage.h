/**
 * @file FieldCoverage.h
 *
 * Declaration to send information about the field coverage.
 *
 * @author Nicole Schrader
 */

#pragma once

#include "Representations/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Streams/AutoStreamable.h"
#include <vector>

STREAMABLE(FieldCoverage, COMMA public BHumanMessageParticle<idFieldCoverage>
{
  /** BHumanMessageParticle functions */
  void operator>>(BHumanMessage& m) const override;
  void operator<<(const BHumanMessage& m) override;
  bool handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp) override;

  STREAMABLE(GridLine,
  {,
    (int) y,
    (std::vector<unsigned>)() timestamps,
  }),

  (std::array<GridLine, 12>) lines,
});
