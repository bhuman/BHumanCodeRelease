/**
 * @file DamageConfiguration.h
 * Provides data about disabling some functions because of hardware failures.
 *
 * @author Benjamin Markowsky
 */

#pragma once

#include "RobotParts/FsrSensors.h"
#include "RobotParts/Joints.h"
#include "RobotParts/Legs.h"
#include "Streaming/EnumIndexedArray.h"
#include <cstring>
#include "Math/Pose2f.h"
#include "Streaming/Enum.h"

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
  (Joints::stdVectorJoint)(Joints::stdVectorJoint()) jointsToEraseStiffness,
  (Joints::stdVectorJoint)(Joints::stdVectorJoint()) jointsToMuteSirenes,
  (Vector2f) startTiltLeft,
  (Vector2f) startTiltRight,
  (ENUM_INDEXED_ARRAY(Side, Legs::Leg)) sides,
});
