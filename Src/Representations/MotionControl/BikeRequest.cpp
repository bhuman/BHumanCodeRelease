/**
* @file Representations/MotionControl/BikeRequest.cpp
* @author <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
*/

#include <cstring>

#include "BikeRequest.h"

BikeRequest::BMotionID BikeRequest::getBMotionFromName(const char* name)
{
  for(int i = 0; i < numOfBMotionIDs; ++i)
    if(!strcmp(name, getName(BMotionID(i))))
      return BMotionID(i);
  return numOfBMotionIDs;
}
