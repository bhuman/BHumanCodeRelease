/**
 * @file FootSupport.cpp
 *
 * @author Philip Reichenberg
 */

#include "FootSupport.h"
#include "Debugging/Plot.h"

void FootSupport::draw() const
{
  PLOT("representation:FootSupport:support", support);
  PLOT("representation:FootSupport:supportPredictSwitch", predictedSwitched ? 1.f : 0.f);
  PLOT("representation:FootSupport:supportPredictSwitch100", predictedSwitched ? 100.f : 0.f);
  PLOT("representation:FootSupport:supportSwitch", switched ? 1.f : 0.f);
  PLOT("representation:FootSupport:supportSwitch100", switched ? 100.f : 0.f);
}
