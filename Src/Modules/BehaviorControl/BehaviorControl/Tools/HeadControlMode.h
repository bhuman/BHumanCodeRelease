#pragma once

#include "Tools/Streams/Enum.h"

struct HeadControl
{
  ENUM(Mode,
  {,
    none,
    off,
    lookActive,
    lookActiveWithBall,
    lookActiveWithOwnBall,
    lookActiveWithoutBall,
    lookAtBall,
    lookAtBallMirrored,
    lookAtOwnBall,
    lookAtOwnBallMirrored,
    lookAtGlobalBall,
    lookAtGlobalBallMirrored,
    lookForward,
    lookLeftAndRight,
    lookRightAndLeft,
  });
};
using HeadControlMode = HeadControl::Mode;