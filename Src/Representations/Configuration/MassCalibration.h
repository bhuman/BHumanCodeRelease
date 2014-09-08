/**
* @file MassCalibration.h
* Declaration of a class for representing the relative positions and masses of mass points.
* @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
*/

#pragma once

#include "Tools/Math/Vector3.h"
#include "Tools/Enum.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(MassCalibration,
{
public:
  ENUM(Limb,
    neck,
    head,
    shoulderLeft,
    bicepsLeft,
    elbowLeft,
    foreArmLeft,
    shoulderRight,
    bicepsRight,
    elbowRight,
    foreArmRight,
    pelvisLeft,
    hipLeft,
    thighLeft,
    tibiaLeft,
    ankleLeft,
    footLeft,
    pelvisRight,
    hipRight,
    thighRight,
    tibiaRight,
    ankleRight,
    footRight,
    torso
  );

  /**
  * Information on the mass distribution of a limb of the robot.
  */
  STREAMABLE(MassInfo,
  {,
    (float)(0) mass, /**< The mass of this limb. */
    (Vector3<>) offset, /**< The offset of the center of mass of this limb relative to its hinge. */
  }),

  (MassInfo[numOfLimbs]) masses, /**< Information on the mass distribution of all joints. */
});
