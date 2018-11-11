/**
 * @file Representations/MotionControl/SpecialActionRequest.cpp
 * This file implements a struct to represent special action requests.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#include <cstring>

#include "SpecialActionRequest.h"

SpecialActionRequest::SpecialActionID SpecialActionRequest::getSpecialActionFromName(const char* name)
{
  FOREACH_ENUM(SpecialActionID, i)
    if(!strcmp(name, TypeRegistry::getEnumName(i)))
      return i;
  return numOfSpecialActionIDs;
}
