/**
 * @file KickEngineParameters.h
 * @author <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
 */

#pragma once

#define NUM_OF_POINTS 3

#include <vector>

#include "Tools/Math/Eigen.h"
#include "Tools/Enum.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(DynPoint,
{
  DynPoint() = default;
  DynPoint(int limb, int phaseNumber, int duration, const Vector3f& translation, const Vector3f& angle, const Vector3f& odometryOffset);
  DynPoint(int limb, int phaseNumber, const Vector3f& translation);

  bool operator==(const DynPoint& other) const
  {
    return limb == other.limb &&
           phaseNumber == other.phaseNumber &&
           duration == other.duration &&
           translation == other.translation &&
           angle == other.angle &&
           odometryOffset == other.odometryOffset;
  },

  (int) limb,
  (int) phaseNumber,
  (int) duration,
  (Vector3f)(Vector3f::Zero()) translation,
  (Vector3f)(Vector3f::Zero()) angle,
  (Vector3f)(Vector3f::Zero()) odometryOffset,
});

inline DynPoint::DynPoint(int limb, int phaseNumber, int duration, const Vector3f& translation, const Vector3f& angle, const Vector3f& odometryOffset) :
  limb(limb),
  phaseNumber(phaseNumber),
  duration(duration),
  translation(translation),
  angle(angle),
  odometryOffset(odometryOffset)
{}

inline DynPoint::DynPoint(int limb, int phaseNumber, const Vector3f& translation) :
  limb(limb),
  phaseNumber(phaseNumber),
  duration(0), //no change
  translation(translation)
{}

class Phase : public Streamable
{
public:
  ENUM(Limb,
  {,
    leftFootTra,
    leftFootRot,
    rightFootTra,
    rightFootRot,
    leftArmTra,
    leftHandRot,
    rightArmTra,
    rightHandRot,
  });

  unsigned int duration;

  Vector3f controlPoints[Phase::numOfLimbs][NUM_OF_POINTS];
  Vector2f comTra[NUM_OF_POINTS];
  Vector2f headTra[NUM_OF_POINTS];

  Vector3f originPos[Phase::numOfLimbs];
  Vector2f comOriginPos = Vector2f::Zero();
  Vector2f comOriginOffset = Vector2f::Zero();
  Vector2f headOrigin = Vector2f::Zero();
  Vector3f odometryOffset = Vector3f::Zero();

  virtual void serialize(In* in, Out* out);
};

class KickEngineParameters : public Streamable
{
public:
  int numberOfPhases;
  float preview;
  bool loop, autoComTra, ignoreHead;
  char name[260];

  Vector3f footOrigin = Vector3f::Zero();
  Vector3f armOrigin = Vector3f::Zero();
  Vector3f footRotOrigin = Vector3f::Zero();
  Vector3f handRotOrigin = Vector3f::Zero();
  Vector2f comOrigin = Vector2f::Zero();
  Vector2f headOrigin = Vector2f::Zero();

  std::vector<Phase> phaseParameters;
  float kpx, kdx, kix, kpy, kdy, kiy;

  /**
  * The method serializes this object.
  * @param in Points to an In-stream, when currently reading.
  * @param out Points to an Out-stream, when currently writing.
  */
  virtual void serialize(In* in, Out* out);

  void calcControlPoints();

  Vector3f getPosition(const float& phase, const int& phaseNumber, const int& limb);

  Vector2f getComRefPosition(const float& phase, const int& phaseNumber);
  Vector2f getHeadRefPosition(const float& phase, const int& phaseNumber);

  void initFirstPhase();
  void initFirstPhase(Vector3f* origins, Vector2f head);
  void initFirstPhaseLoop(Vector3f *origins, Vector2f lastCom, Vector2f head);

  KickEngineParameters() :
    numberOfPhases(0),
    preview(150),
    loop(false),
    autoComTra(false),
    ignoreHead(false),
    phaseParameters(0),
    kpx(0.f),
    kdx(0.f),
    kix(0.f),
    kpy(0.f),
    kdy(0.f),
    kiy(0.f)
  {}
};
