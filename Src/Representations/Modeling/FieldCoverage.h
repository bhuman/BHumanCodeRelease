/**
 * @file FieldCoverage.h
 * @author Andreas Stolpmann
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include <vector>
#include "Representations/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"

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

  (int)(0) lineToSendNext,
  (std::vector<GridLine>) lines,
});
