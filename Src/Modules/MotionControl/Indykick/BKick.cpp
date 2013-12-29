/**
 * @file BKick.cpp
 * Implementation of kick motion methods.
 * @author Felix Wenk
 */

#include "BKick.h"

StandSupportLegPhase::StandSupportLegPhase(const int durationInteger, const BKickPhase::PhaseId phaseId)
  : BKickPhase(phaseId, durationInteger), zmpCurve(Vector2f(), Vector2f(), Vector2f(), Vector2f())
{}

BKickPhase *StandSupportLegPhase::clone() const
{
  StandSupportLegPhase *object = new StandSupportLegPhase(defaultDuration, phaseId);
  object->initialKickFootPoseRotation = initialKickFootPoseRotation;
  object->initialKickFootPoseTranslation = initialKickFootPoseTranslation;
  object->standKickFootPoseRotation = standKickFootPoseRotation;
  object->standKickFootPoseTranslation = standKickFootPoseTranslation;
  object->zmpCurve = zmpCurve;
  return object;
}

void StandSupportLegPhase::seek(const float pos, KickFootPose& kickFoot) const
{
  ASSERT(kickFoot.reference == KickFootPose::supportFoot);
  kickFoot.pose.translation = initialKickFootPoseTranslation * (1.0f - pos) + standKickFootPoseTranslation * pos;
  kickFoot.pose.rotation = RotationMatrix(initialKickFootPoseRotation * (1.0f - pos) + standKickFootPoseRotation * pos);
}

void StandSupportLegPhase::initialKickFootPose(const KickFootPose& kickFoot)
{
  ASSERT(kickFoot.reference == KickFootPose::supportFoot);
  initialKickFootPoseTranslation = kickFoot.pose.translation;
  initialKickFootPoseRotation = kickFoot.pose.rotation.getAngleAxis();
}

void StandSupportLegPhase::reset(const KickFootPose& standKickFootPose)
{
  ASSERT(standKickFootPose.reference == KickFootPose::supportFoot);
  standKickFootPoseTranslation = standKickFootPose.pose.translation;
  standKickFootPoseRotation = standKickFootPose.pose.rotation.getAngleAxis();
}

void StandSupportLegPhase::initialZmp(const Vector2f& initialZmp, const Vector2f& wantedZmp)
{
  zmpCurve.p0 = initialZmp;
  zmpCurve.p1 = zmpCurve.p0;
  zmpCurve.p2 = wantedZmp;
  zmpCurve.p3 = zmpCurve.p2;
}

Vector2f StandSupportLegPhase::zmpAtFutureFrame(unsigned time, const Vector2f& defaultZmp) const
{
  if(defaultDuration < 0)
    return defaultZmp;

  ASSERT(time <= static_cast<unsigned>(defaultDuration));

  const float duration = static_cast<float>(defaultDuration);
  const float time2    = static_cast<float>(time);
  const float position = time2 / duration;

  ASSERT(0.0f <= position && position <= 1.0f);

  Vector2f zmp;
  zmpCurve.evaluate(position, zmp);
  return zmp;
}

KickFootBezierPhase::KickFootBezierPhase(const int durationInteger, BSplineGenerator& generator,
                                         const IndykickRequest& request, const BKickPhase::PhaseId phaseId)
: BKickPhase(phaseId, durationInteger), bezierCurveGenerator(generator), request(request)
{}

void KickFootBezierPhase::seek(const float pos, KickFootPose& kickFoot) const
{
  const BezierCurveReference positions = bezierCurveGenerator.getPositionsReference(phaseId, kickFoot.reference);
  positions.evaluate(pos, kickFoot.pose.translation);
  KickFootPose::Reference referenceCoordinateSystem;
  const AngleBezierCurveReference rotations = bezierCurveGenerator.getAnglesReference(phaseId, referenceCoordinateSystem);
  rotations.evaluate(pos, kickFoot.pose.rotation);
  ASSERT(referenceCoordinateSystem == kickFoot.reference);
}

BKickPhase *KickFootBezierPhase::clone() const
{
  return new KickFootBezierPhase(defaultDuration, bezierCurveGenerator, request, phaseId);
}

void KickFootBezierPhase::initialKickFootPose(const KickFootPose& kickFoot)
{
  bezierCurveGenerator.generateStartpoints(phaseId, kickFoot);
}

void KickFootBezierPhase::reset(const KickFootPose& standKickFootPose)
{
  bezierCurveGenerator.generateEndpoints(phaseId, request, standKickFootPose);
}

int KickFootBezierPhase::getDuration() const
{
  return bezierCurveGenerator.modifiesDuration() ? bezierCurveGenerator.getDuration(phaseId) : defaultDuration;
}

BKick::BKick(unsigned numPhases, const BKickPhase **_phases)
  : numPhases(numPhases), phases(new BKickPhase *[numPhases]), phaseIdx(numPhases - 1)
{
  for(unsigned i = 0; i < numPhases; ++i)
    phases[i] = _phases[i]->clone();
}

BKick::BKick() : numPhases(0), phases(0), phaseIdx(0) {}

BKick::~BKick()
{
  if(phases)
  {
    for(unsigned i = 0; i < numPhases; ++i)
      if(phases[i])
        delete phases[i];
    delete[] phases;
    phases = 0;
  }
}

void BKick::reset(const KickFootPose& kickFoot, const Vector2f& initialZmp, const Vector2f& wantedZmp)
{
  for(unsigned i = 0; i < numPhases; ++i)
    phases[i]->reset(kickFoot);

  phaseIdx = 0;
  phases[phaseIdx]->initialKickFootPose(kickFoot);
  phases[phaseIdx]->initialZmp(initialZmp, wantedZmp);
  phases[numPhases - 1]->initialZmp(wantedZmp, initialZmp);
}

void BKick::nextPhase(const KickFootPose& kickFoot)
{
  phaseIdx = (phaseIdx + 1) % numPhases;
  phases[phaseIdx]->initialKickFootPose(kickFoot);
}

BKickPhase& BKick::currentPhase() const
{
  return *phases[phaseIdx];
}

BKick& BKick::operator=(const BKick& other)
{
  if(phases)
    delete[] phases;
  numPhases = other.numPhases;
  phases = new BKickPhase *[numPhases];
  for(unsigned i = 0; i < numPhases; ++i)
    phases[i] = other.phases[i]->clone();
  phaseIdx = other.phaseIdx;

  return *this;
}

Vector2f BKick::zmpAtFutureFrame(unsigned frame, unsigned timeInPhase,
                                 unsigned cycleTimeMs, const Vector2f& defaultZmp) const
{
  unsigned phaseWithFrame = phaseIdx;
  int time = frame * cycleTimeMs;
  int duration = phases[phaseWithFrame]->getDuration();
  if(duration < 0)
    return phases[phaseWithFrame]->zmpAtFutureFrame(time + timeInPhase, defaultZmp);

  duration -= timeInPhase;
  if(duration <= 0) // Catch duration rouding error and completed phases.
  {
    phaseWithFrame += 1;
    if(phaseWithFrame == numPhases)
      goto endofkick;
    duration = phases[phaseWithFrame]->getDuration();
    timeInPhase = 0;
  }

  while(duration >= 0 && time > duration)
  {
    time -= duration;
    phaseWithFrame += 1;
    if(phaseWithFrame == numPhases)
      goto endofkick;
    duration = phases[phaseWithFrame]->getDuration();
    timeInPhase = 0;
  }

  return phases[phaseWithFrame]->zmpAtFutureFrame(time + timeInPhase, defaultZmp);

endofkick:
  duration = phases[numPhases - 1]->getDuration();
  if(duration < 0)
    duration = 0;
  return phases[numPhases - 1]->zmpAtFutureFrame(duration, defaultZmp);
}
