/**
 * @file Representations/MotionControl/SpecialActionRequest.cpp
 * This file implements a struct to represent special action requests.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#include <cstring>

#include "SpecialActionRequest.h"

SpecialActionRequest::SpecialActionID SpecialActionRequest::getSpecialActionFromName(const char* name)
{
  for(int i = 0; i < numOfSpecialActionIDs; ++i)
    if(!strcmp(name, getName(SpecialActionID(i))))
      return SpecialActionID(i);
  return numOfSpecialActionIDs;
}
