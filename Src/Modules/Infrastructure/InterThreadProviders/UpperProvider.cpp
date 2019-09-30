/**
 * @file UpperProvider.cpp
 *
 * This file implements a module that provides representations from the upper
 * camera thread for the current frame.
 *
 * @author Thomas Röfer
 */

#include "UpperProvider.h"

MAKE_MODULE(UpperProvider, infrastructure)

void UpperProvider::update(OtherFieldBoundary& theOtherFieldBoundary)
{
  static_cast<FieldBoundary&>(theOtherFieldBoundary) = theUpperFieldBoundary;
}

void UpperProvider::update(OtherObstaclesPerceptorData& theOtherObstaclesPerceptorData)
{
  static_cast<ObstaclesPerceptorData&>(theOtherObstaclesPerceptorData) = theUpperObstaclesPerceptorData;
  theOtherObstaclesPerceptorData.imageCoordinateSystem.cameraInfo = theUpperObstaclesPerceptorData.cameraInfo;
}
