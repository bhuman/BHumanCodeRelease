/**
 * @file Interception.h
 *
 * This file declares an enumeration of interception methods.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/Streams/Enum.h"

namespace Interception
{
  ENUM(Method,
  {,
    stand,
    genuflectFromSitting,
    genuflectStandDefender,
    genuflectStand,
    jumpLeft,
    jumpRight,
    walk,
  });
}
