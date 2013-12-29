/**
 * @file BKickBase.h
 * Declarations of base classes for kicking motions.
 * @author Felix Wenk
 */

#pragma once

#include "Tools/Enum.h"
#include "Tools/Math/Pose3D.h"
#include "Tools/Math/Vector.h"

class KickFootPose
{
public:
  ENUM(Reference,
       supportFoot,
       origin);

  KickFootPose() : reference(supportFoot) {}
  KickFootPose(const Pose3D& kickFootPose, const Reference referenceSystem = supportFoot)
  : reference(referenceSystem), pose(kickFootPose) {}

  Reference reference;
  Pose3D pose;
};

class BKickPhase
{
public:
  ENUM(PhaseId,
       uninitialized,
       supportLegStand,
       liftKickFoot,
       strikeoutKickFoot,
       bezierKickFoot,
       ballContact,
       swingOut,
       lowerKickFoot,
       finished);

  BKickPhase();
  BKickPhase(const PhaseId phaseId, const int duration);
  /**
   * Virtual destructor to avoid warning.
   * Not really needed, since subclasses don't do any cleanup.
   */
  virtual ~BKickPhase() {}

  /**
   * Gets the kick foot pose (relative to the support foot) corresponding
   * to the current time in the phase. 0 marks the start of the phase, the phase duration the end.
   * (so 0 <= timeInPhase <= phaseDuration)
   *
   * returns true if the phase is completed.
   */
  bool seek(const unsigned timeInPhase, KickFootPose& kickFoot) const;

  /**
   * Gets the kick foot pose (relative to the support foot) corresponding
   * to the current position in the phase. 0 marks the start of the phase, 1 the end.
   * (so 0 <= pos <= 1)
   */
  virtual void seek(const float pos, KickFootPose& kickFoot) const = 0;
  virtual BKickPhase *clone() const = 0;

  /**
   * Sets the initial kick foot pose in support foot coordinates.
   */
  virtual void initialKickFootPose(const KickFootPose& kickFoot) = 0;

  /**
   * Sets the initial zero moment point and the wanted zmp.
   */
  virtual void initialZmp(const Vector2f& initialZmp, const Vector2f& wantedZmp);

  /**
   * Resets this phase. Since most phases do not need resetting,
   * this method has an empty default implementation. This method
   * is meant to be called when the kick containing this phase is
   * being reset.
   */
  virtual void reset(const KickFootPose& standKickFootPose);

  virtual int getDuration() const;

  virtual Vector2f zmpAtFutureFrame(unsigned time, const Vector2f& defaultZmp) const;

  PhaseId phaseId;
protected:
  int defaultDuration; /**< Default duration of the phase in milliseconds. */
};
