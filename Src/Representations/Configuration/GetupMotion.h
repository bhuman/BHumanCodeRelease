/**
 * @file GetUpMotion.h
 * Contains the List of all available GetUpMotions defined in getUpEngine.cfg
 *
 * @author Bernd Poppinga
 */

#pragma once

#include "Tools/Streams/Enum.h"

namespace GetUpMotions
{
  ENUM(GetUpMotion,
  {, //all motions defined in getUpEngine.cfg
    frontFast,
    backFast,
    backGenu,
    recoverAndWait,
    recoverFast,
    recoverAfterBadBreakUp,
    recoverFromSide,
    recoverFromSideAfterJump,
    recoverFastKeeperJump,
    recoverBadArms,
    stand,
    fromGenuflect,
    fromSitDown,
    fromSumo,
    backFreeJoints,
    frontFreeJoints,
    sitDownFreeJoints,
    backVeryFast,
  });

  using GetupMotionVector = std::vector<GetUpMotion>;
}
