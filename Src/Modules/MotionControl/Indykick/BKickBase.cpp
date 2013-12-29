/**
 * @file BKickBase.cpp
 * Implementation of kick motion base class methods.
 * @author Felix Wenk
 */

#include "Tools/Debugging/Asserts.h"
#include "Tools/Debugging/DebugDrawings.h"

#include "BKickBase.h"

BKickPhase::BKickPhase()
: phaseId(uninitialized),
defaultDuration(-1)
{}

BKickPhase::BKickPhase(const PhaseId phaseId, const int durationInteger)
: phaseId(phaseId),
defaultDuration(durationInteger)
{}

bool BKickPhase::seek(const unsigned timeInPhase, KickFootPose& kickFoot) const
{
  const int duration = getDuration();
  bool completed = duration < 0 ? false : timeInPhase >= static_cast<unsigned>(duration);
  float pos = 0.0f;
  if(completed)
  {
    pos = 1.0f;
  }
  else if(timeInPhase == 0)
  {
    pos = 0.0f;
  }
  else
  {
    const float t = static_cast<float>(timeInPhase);
    const float durationFloat = static_cast<float>(duration);
    pos = t / durationFloat;
  }

  PLOT("module:IndykickEngine:phaseCompletion", pos * 20.0f);

  seek(pos, kickFoot);
  return completed;
}

void BKickPhase::initialZmp(const Vector2f &initialZmp, const Vector2f &wantedZmp) {}

void BKickPhase::reset(const KickFootPose& standKickFootPose) {}

int BKickPhase::getDuration() const
{
  return defaultDuration;
}


Vector2f BKickPhase::zmpAtFutureFrame(unsigned int time, const Vector2f& defaultZmp) const
{
  return defaultZmp;
}
