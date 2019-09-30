#include "LegMotionSelection.h"
#include "Tools/Debugging/DebugDrawings.h"

#include <numeric>

void LegMotionSelection::draw()
{
  PLOT("representation:MotionSelection:walk", ratios[MotionRequest::walk]);
  PLOT("representation:MotionSelection:kick", ratios[MotionRequest::kick]);
  PLOT("representation:MotionSelection:specialAction", ratios[MotionRequest::specialAction]);
  PLOT("representation:MotionSelection:stand", ratios[MotionRequest::stand]);
  PLOT("representation:MotionSelection:getUp", ratios[MotionRequest::getUp]);

  const float sum = std::accumulate(ratios.begin(), ratios.end(), 0.f);
  PLOT("representation:MotionSelection:sum", sum);
}
