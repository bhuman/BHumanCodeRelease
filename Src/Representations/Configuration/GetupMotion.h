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
  GLOBAL_ENUM(GetUpMotion,
  {, //all motions defined in getUpEngine.cfg
    front,
    frontFast,
    frontSlow,
    back,
    backGenu,
    backFast,
    backSlow,
    backOld,
    recoverAndWait,
    recoverFast,
    stand,
  });

  using GetupMotionVector = std::vector<GetUpMotion>;
}
