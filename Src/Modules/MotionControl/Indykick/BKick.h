/**
 * @file BKick.h
 * Declarations of kicking motions.
 * @author Felix Wenk
 */

#pragma once

#include "Tools/Math/Vector.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/MotionControl/IndykickRequest.h"
#include "BKickBase.h"
#include "BezierBase.h"
#include "BSplineGenerator.h"

class StandSupportLegPhase : public BKickPhase
{
public:
  StandSupportLegPhase(const int duration, const BKickPhase::PhaseId phaseId = BKickPhase::supportLegStand);

  void seek(const float pos, KickFootPose& kickFoot) const;
  BKickPhase *clone() const;
  /**
   * Sets the initial kick foot pose.
   */
  void initialKickFootPose(const KickFootPose& kickFoot);
  void reset(const KickFootPose& standKickFootPose);
  void initialZmp(const Vector2f& initialZmp, const Vector2f& wantedZmp);
  Vector2f zmpAtFutureFrame(unsigned time, const Vector2f& defaultZmp) const;

private:
  /** The initial pose of the kick foot in support-foot-coordinates. */
  Vector3<> initialKickFootPoseTranslation;
  Vector3<> initialKickFootPoseRotation;
  /** The stand pose of the kick foot in support-foot-coordinates. */
  Vector3<> standKickFootPoseTranslation;
  Vector3<> standKickFootPoseRotation;
  BezierCurveTemplate<Vector2f> zmpCurve;
};

class KickFootBezierPhase : public BKickPhase
{
public:
  KickFootBezierPhase(const int duration, BSplineGenerator& generator,
                      const IndykickRequest& request, const BKickPhase::PhaseId phaseId = bezierKickFoot);
  void seek(const float pos, KickFootPose& kickFoot) const;
  BKickPhase *clone() const;
  void initialKickFootPose(const KickFootPose& kickFoot);
  void reset(const KickFootPose& standKickFootPose);
  int getDuration() const;
private:
  BSplineGenerator& bezierCurveGenerator;
  const IndykickRequest& request; /**< The request due to which this phase is currenlty executed. */
};


class BKick
{
public:
  BKick();
  /**
   * Constructor.
   * _phases is an array of pointers to BKickPhase objects. The BKickPhase
   * objects are copied by this constructor.
   */
  BKick(unsigned phases, const BKickPhase **_phases);
  ~BKick();
  /* reset and nextPhase both take the current pose of the kick foot in
     support foot coordinates. */
  void reset(const KickFootPose& kickFoot, const Vector2f& initialZmp, const Vector2f& wantedZmp);
  void nextPhase(const KickFootPose& kickFoot);
  BKickPhase& currentPhase() const;
  BKick& operator=(const BKick& other);
  int calculateZmpTrajectory(Vector2f *zmpRef, const int start, const int end, const Vector2f& defaultZmp, const Vector2f& balanceRegion, const int timeInCurrentPhase);
  Vector2f zmpAtFutureFrame(unsigned frame, unsigned timeInPhase,
                            unsigned cycleTimeMs, const Vector2f& defaultZmp) const;

private:
  unsigned numPhases;
  /**
   * An array of pointers to the phases. The phase objects are
   * owned by this class, i.e. will be deleted when this BKick
   * object is deleted.
   */
  BKickPhase **phases;
  unsigned phaseIdx;
};
