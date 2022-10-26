/**
 * @file Interception.h
 *
 * This file declares an enumeration of interception methods.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Streaming/Enum.h"

namespace Interception
{
  ENUM(Method,
  {,
    stand,
    walk,
    genuflectStandDefender,
    genuflectStand,
    jumpLeft,
    jumpRight,
  });
}
