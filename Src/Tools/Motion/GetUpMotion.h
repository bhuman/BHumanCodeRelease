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
    doNothing, //Default, to prevent damage of robots if the configuration is wrong.
    front,
    back,
    fromSplit,
    recoverFast,
    recoverFromSideBack,
    recoverFromSideFront,
    recoverArmLeftFrontLyingOn,
    stand,
    sit,
  });

  using GetUpMotionVector = std::vector<GetUpMotion>;
}
