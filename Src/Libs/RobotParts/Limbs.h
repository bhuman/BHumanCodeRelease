#pragma once

#include "RobotParts/Arms.h"
#include "RobotParts/Legs.h"
#include "Streaming/Enum.h"

namespace Limbs
{
  ENUM(Limb,
  {,
    neck,
    head,

    firstLeftArmLimb,

    shoulderLeft = firstLeftArmLimb,
    bicepsLeft,
    elbowLeft,
    foreArmLeft,
    wristLeft,

    firstRightArmLimb,

    shoulderRight = firstRightArmLimb,
    bicepsRight,
    elbowRight,
    foreArmRight,
    wristRight,

    firstLeftLegLimb,

    pelvisLeft = firstLeftLegLimb,
    hipLeft,
    thighLeft,
    tibiaLeft,
    ankleLeft,
    footLeft,

    firstRightLegLimb,

    pelvisRight = firstRightLegLimb,
    hipRight,
    thighRight,
    tibiaRight,
    ankleRight,
    footRight,

    torso,
    waist,
  });

  ENUM(LimbArmVarieties,
  {,
    shoulder,
    biceps,
    elbow,
    foreArm,
    wrist,
  });

  ENUM(LimbLegVarieties,
  {,
    pelvis,
    hip,
    thigh,
    tibia,
    ankle,
    foot,
  });

  inline Limb combine(const Arms::Arm arm, const LimbArmVarieties limbV)
  {
    static const unsigned offset[2] = { 0u, firstRightArmLimb - firstLeftArmLimb };
    return Limb(firstLeftArmLimb + offset[arm] + limbV);
  }

  inline Limb combine(const Legs::Leg leg, const LimbLegVarieties limbV)
  {
    static const unsigned offset[2] = { 0u, firstRightLegLimb - firstLeftLegLimb };
    return Limb(firstLeftLegLimb + offset[leg] + limbV);
  }
}
