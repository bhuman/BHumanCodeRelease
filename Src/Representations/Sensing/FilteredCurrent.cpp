/**
 * @file FilteredCurrent.cpp
 *
 * @author Philip Reichenberg
 */

#include "FilteredCurrent.h"
#include "Debugging/Plot.h"

void FilteredCurrent::draw() const
{
  PLOT("representation:FilteredCurrent:lHYP", currents[Joints::lHipYawPitch]);
  PLOT("representation:FilteredCurrent:lHP", currents[Joints::lHipPitch]);
  PLOT("representation:FilteredCurrent:lKP", currents[Joints::lKneePitch]);
  PLOT("representation:FilteredCurrent:lAP", currents[Joints::lAnklePitch]);
  PLOT("representation:FilteredCurrent:rHP", currents[Joints::rHipPitch]);
  PLOT("representation:FilteredCurrent:rKP", currents[Joints::rKneePitch]);
  PLOT("representation:FilteredCurrent:rAP", currents[Joints::rAnklePitch]);
  PLOT("representation:FilteredCurrent:lSum", legPitchCurrentSums[Legs::left]);
  PLOT("representation:FilteredCurrent:rSum", legPitchCurrentSums[Legs::right]);
}
