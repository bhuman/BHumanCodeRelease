/**
 * @file JointPlayTranslation.h
 * @author Philip Reichenberg
 */

#pragma once

#include "Platform/BHAssert.h"
#include "RobotParts/Joints.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/EnumIndexedArray.h"
#include "Math/Range.h"

STREAMABLE(JointPlayTranslation,
{
  STREAMABLE(JointPlayState,
  {,
    (Angle)(0) offset,
    (Angle)(0) play,
  });

  void draw() const;
  JointPlayTranslation(),

  (ENUM_INDEXED_ARRAY(Rangea, Joints::Joint)) jointOffset,
  (ENUM_INDEXED_ARRAY(JointPlayState, Joints::Joint)) jointPlayState,
});

inline JointPlayTranslation::JointPlayTranslation()
{
  jointOffset.fill({0, 0});
}
