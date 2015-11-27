/**
 * @file Representations/MotionControl/KickRequest.cpp
 * @author <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
 */

#include <cstring>

#include "KickRequest.h"

KickRequest::KickMotionID KickRequest::getKickMotionFromName(const char* name)
{
  for(int i = 0; i < numOfKickMotionIDs; ++i)
    if(!strcmp(name, getName(KickMotionID(i))))
      return KickMotionID(i);
  return numOfKickMotionIDs;
}
