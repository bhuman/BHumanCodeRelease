/**
 * @file Representations/MotionControl/KickRequest.cpp
 * @author <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
 */

#include <cstring>

#include "KickRequest.h"

KickRequest::KickMotionID KickRequest::getKickMotionFromName(const char* name)
{
  FOREACH_ENUM(KickMotionID, i)
    if(!strcmp(name, TypeRegistry::getEnumName(i)))
      return i;
  return numOfKickMotionIDs;
}
