/**
 * @file Representations/MotionControl/KickRequest.h
 * @author <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
 */

#pragma once
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(DynPoint,
{
  DynPoint() = default;
  DynPoint(int limb, int phaseNumber, int duration, const Vector3f& translation, const Vector3f& angle, const Vector3f& odometryOffset);
  DynPoint(int limb, int phaseNumber, const Vector3f& translationl, const int duration = -1);

  bool operator==(const DynPoint& other) const,

  (int) limb,
  (int) phaseNumber,
  (int)(0) duration,
  (Vector3f)(Vector3f::Zero()) translation,
  (Vector3f)(Vector3f::Zero()) angle,
  (Vector3f)(Vector3f::Zero()) odometryOffset,
});

inline bool DynPoint::operator==(const DynPoint& other) const
{
  return limb == other.limb &&
         phaseNumber == other.phaseNumber &&
         duration == other.duration &&
         translation == other.translation &&
         angle == other.angle &&
         odometryOffset == other.odometryOffset;
};

inline DynPoint::DynPoint(int limb, int phaseNumber, int duration, const Vector3f& translation,
                          const Vector3f& angle, const Vector3f& odometryOffset) :
  limb(limb), phaseNumber(phaseNumber), duration(duration),
  translation(translation), angle(angle), odometryOffset(odometryOffset)
{}

inline DynPoint::DynPoint(int limb, int phaseNumber, const Vector3f& translation, const int duration) :
  limb(limb), phaseNumber(phaseNumber), duration(duration), translation(translation)
{}

STREAMABLE(KickRequest,
{
  ENUM(KickMotionID,
  {,
    kickForward,
    // kicks up to here are loaded by the KickEngine
    newKick,
    none,
  });

  static KickMotionID getKickMotionFromName(const char* name),

  (KickMotionID)(none) kickMotionType,
  (bool)(false) mirror,
  (bool)(false) armsBackFix,
  (bool)(false) autoProceed,
  (bool)(false) boost,
  (std::vector<DynPoint>) dynPoints,
});

STREAMABLE(Continuation,
{,
  (KickRequest::KickMotionID)(KickRequest::none) kickType,
  (bool)(false) mirror,
});

using stdVectorContinuation = std::vector<Continuation>;
