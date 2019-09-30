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
    allFine, /**< normal and mirrored get up is allowed */
    onlyNormal, /**< only normal get up is allowed */
    onlyMirrored, /**< only mirrored get up is allowed */
    allBroken, /**< don't even try to stand */
  });

  STREAMABLE(Side,
  {,
    (bool)(false) weakLeg,
    (bool)(false) footBumperDefect,
    (bool)(false) armContactDefect,
  }),
  (bool)(false) noFieldGenuflect,
  (BrokenStandUp)(allFine) brokenStandUp,
  (float)(0) optionalLineVersionFront,
  (float)(0) optionalLineVersionBack,
  (Joints::stdVectorJoint)(Joints::stdVectorJoint()) jointsToEraseStiffness,
  (Vector2f) startTiltLeft,
  (Vector2f) startTiltRight,
  (bool) dontBoost,
  (ENUM_INDEXED_ARRAY(Side, Legs::Leg)) sides,
});
