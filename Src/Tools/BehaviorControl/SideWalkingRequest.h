/**
 * @file SideWalkingRequest.h
 * This file contains information about how the walk speed be reduced
 *
 * @author Philip Reichenberg
 */

#pragma once
#include "Streaming/Enum.h"

namespace SideWalkingRequest
{
  ENUM(SideWalkingRequest,
  {,
    allowed, /**< No change. Motion can decided by itself. */
    forced, /**< Force side walking. */
    notAllowed, /**< Side walking is not allowed. */
  });
}
