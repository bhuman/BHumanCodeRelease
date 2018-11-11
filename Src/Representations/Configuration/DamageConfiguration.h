/**
 * @file DamageConfiguration.h
 * Provides data about disabling some functions because of hardware failures.
 *
 * @author Benjamin Markowsky
 */

#pragma once

#include "Tools/RobotParts/FsrSensors.h"
#include "Tools/RobotParts/Joints.h"
#include "Tools/RobotParts/Legs.h"
#include "Tools/Streams/EnumIndexedArray.h"
#include <cstring>
#include "Tools/Math/Pose2f.h"
#include "Tools/Streams/Enum.h"
#include "GetupMotion.h"

STREAMABLE(DamageConfigurationHead,
{
  DamageConfigurationHead()
  {
    std::memset(audioChannelsDefect, 0, sizeof(audioChannelsDefect));
  },

  (bool[4]) audioChannelsDefect,
});

STREAMABLE(DamageConfigurationBody,
{
  ENUM(BrokenStandUp,
  {,
    allFine, /**< try left and if not successful right. */
    onlyNormal, /**< left stand foot based */
    onlyMirrored, /**< right stand foot based */
    allBroken, /**< don't even try to stand. */
  });

  STREAMABLE(Side,
  {
    Side()
    {
      brokenFsrs.fill(false);
    },

    (bool)(false) weakLeg,
    (bool)(false) footBumperDefect,
    (bool)(false) armContactDefect,
    (ENUM_INDEXED_ARRAY(bool, FsrSensors::FsrSensor)) brokenFsrs,
  }),
  (bool)(false) noFieldGenuflect,
  (BrokenStandUp)(allFine) brokenStandUp,
  (GetUpMotions::GetupMotionVector) getUpBack,
  (GetUpMotions::GetupMotionVector) getUpFront,
  (float)(0) optionalLineVersionFront,
  (float)(0) optionalLineVersionBack,
  (Joints::stdVectorJoint)(Joints::stdVectorJoint()) jointsToEraseStiffness,
  (Vector2f) startTiltLeft,
  (Vector2f) startTiltRight,
  (ENUM_INDEXED_ARRAY(Side, Legs::Leg)) sides,
});
